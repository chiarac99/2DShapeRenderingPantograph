"""
SVG curvature-weighted point sampler.

Reads an SVG containing path outlines (stroke width ~1), reorders the paths
into a continuous chain, merges them into a single global polyline, then
samples a specified number of points with density proportional to curvature.
Because curvature is estimated globally, sharp corners at path junctions are
naturally captured. Saves coordinates to a CSV and shows a matplotlib preview.

Dependencies:
    pip install svgpathtools numpy matplotlib scipy
"""

import csv
import argparse
import numpy as np
from scipy.ndimage import gaussian_filter1d
import svgpathtools
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path


# ---------------------------------------------------------------------------
# 1. Path reordering — sort into a continuous chain
# ---------------------------------------------------------------------------

def reorder_paths(paths, gap_threshold=1.0):
    """
    Sort paths into a continuous chain so that the endpoint of each path is
    as close as possible to the startpoint of the next. Each path can also be
    reversed if that gives a better join.

    Uses a greedy nearest-neighbour approach: starting from the first path,
    repeatedly find the unused path whose start or end is closest to the
    current chain tip, flip it if needed, and append it.

    Parameters
    ----------
    paths         : list of svgpathtools Path objects (any order)
    gap_threshold : distance (SVG units) below which a join is considered
                    connected. Gaps larger than this are reported as warnings.

    Returns
    -------
    ordered_paths : list of Path objects in chain order, flipped where needed
    """
    if not paths:
        return []

    remaining = list(paths)
    ordered = [remaining.pop(0)]

    while remaining:
        # Tip = endpoint of the last path in the chain
        tip = ordered[-1].point(1)

        best_idx = None
        best_dist = float("inf")
        best_flip = False

        for i, path in enumerate(remaining):
            d_start = abs(path.point(0) - tip)
            d_end   = abs(path.point(1) - tip)
            if d_start < best_dist:
                best_dist, best_idx, best_flip = d_start, i, False
            if d_end < best_dist:
                best_dist, best_idx, best_flip = d_end, i, True

        next_path = remaining.pop(best_idx)

        if best_flip:
            next_path = next_path.reversed()

        if best_dist > gap_threshold:
            print(
                f"  Warning: gap of {best_dist:.2f} SVG units between paths "
                f"-- outline may not be fully connected."
            )

        ordered.append(next_path)

    return ordered


# ---------------------------------------------------------------------------
# 2. Merge all paths into one global polyline
# ---------------------------------------------------------------------------

def merge_paths_to_polyline(paths, n_uniform_per_path=4000):
    """
    Densely sample each path uniformly in arc-length, then concatenate them
    into a single polyline.

    At each path junction the endpoint of one path and the startpoint of the
    next become adjacent samples, so a large direction change between them
    naturally shows up as high curvature when we differentiate.

    Parameters
    ----------
    paths               : list of svgpathtools Path objects, already reordered
    n_uniform_per_path  : number of uniform samples per path

    Returns
    -------
    points   : (N,) complex array of x + iy coordinates
    arc_norm : (N,) array of cumulative arc-length, normalised to [0, 1]
    """
    all_points = []
    all_arc_lengths = []
    cumulative_length = 0.0

    for path in paths:
        path_length = path.length()
        if path_length == 0:
            continue

        t_values = np.linspace(0, 1, n_uniform_per_path)
        local_arc = np.array([path.length(T0=0, T1=t) for t in t_values])
        pts = np.array([path.point(t) for t in t_values])

        # Skip the first point of every path after the first to avoid
        # duplicating the junction point (end of previous == start of next)
        if all_points:
            pts = pts[1:]
            local_arc = local_arc[1:]

        all_points.append(pts)
        all_arc_lengths.append(cumulative_length + local_arc)
        cumulative_length += path_length

    points = np.concatenate(all_points)
    arc_lengths = np.concatenate(all_arc_lengths)
    arc_norm = arc_lengths / arc_lengths[-1]

    return points, arc_norm


# ---------------------------------------------------------------------------
# 3. Curvature estimation
# ---------------------------------------------------------------------------

def compute_curvature(points, smooth_sigma=5.0):
    """
    Estimate discrete curvature at each point using finite differences,
    with Gaussian smoothing applied to suppress numerical noise.
    k = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)

    Parameters
    ----------
    points       : (N,) array of complex numbers
    smooth_sigma : standard deviation of Gaussian smoothing kernel (in samples).
                   Larger values = smoother curvature estimate, less noise.
                   Default 5.0 works well for ~4000 samples per path.

    Returns (N,) array of curvature magnitudes.
    """
    x = points.real
    y = points.imag

    # Smooth the coordinates before differentiating to suppress numerical noise
    xs = gaussian_filter1d(x, sigma=smooth_sigma)
    ys = gaussian_filter1d(y, sigma=smooth_sigma)

    dx  = np.gradient(xs)
    dy  = np.gradient(ys)
    d2x = np.gradient(dx)
    d2y = np.gradient(dy)

    numerator   = np.abs(dx * d2y - dy * d2x)
    denominator = (dx**2 + dy**2) ** 1.5

    with np.errstate(invalid="ignore", divide="ignore"):
        kappa = np.where(denominator > 1e-12, numerator / denominator, 0.0)

    return kappa


# ---------------------------------------------------------------------------
# 4. Curvature-weighted resampling
# ---------------------------------------------------------------------------

def curvature_weighted_sample(points, arc_norm, n_points=200, epsilon=1e-3, smooth_sigma=5.0):
    """
    Resample `n_points` from a polyline defined by (points, arc_norm) with
    density proportional to curvature.

    Parameters
    ----------
    points   : (N,) complex array -- the global polyline
    arc_norm : (N,) float array -- cumulative arc-length normalised to [0, 1]
    n_points : desired number of output points
    epsilon  : curvature floor (fraction of max) so straight sections still
               receive some points

    Returns
    -------
    sampled_points : (n_points,) complex array
    """
    kappa = compute_curvature(points, smooth_sigma=smooth_sigma)

    # Floor so straight segments aren't completely empty
    kappa_floor = epsilon * (kappa.max() if kappa.max() > 0 else 1.0)
    weight = kappa + kappa_floor

    # Build CDF over arc-length weighted by curvature
    weighted_density = weight / np.trapz(weight, arc_norm)
    cdf = np.zeros(len(points))
    cdf[1:] = np.cumsum(
        0.5 * (weighted_density[:-1] + weighted_density[1:]) * np.diff(arc_norm)
    )
    cdf /= cdf[-1]  # ensure it ends exactly at 1

    # Invert CDF at evenly-spaced probability targets
    prob_targets = np.linspace(0, 1, n_points)
    arc_targets  = np.interp(prob_targets, cdf, arc_norm)

    sampled_points = (
        np.interp(arc_targets, arc_norm, points.real)
        + 1j * np.interp(arc_targets, arc_norm, points.imag)
    )
    return sampled_points


# ---------------------------------------------------------------------------
# 5. Load SVG paths
# ---------------------------------------------------------------------------

def load_paths(svg_file):
    """
    Load all paths from an SVG file.
    Returns (paths, attributes, canvas_width, canvas_height).
    """
    paths, attributes, svg_attributes = svgpathtools.svg2paths2(str(svg_file))

    w = svg_attributes.get("width", "500")
    h = svg_attributes.get("height", "500")
    viewBox = svg_attributes.get("viewBox", None)

    if viewBox:
        parts = viewBox.replace(",", " ").split()
        w, h = parts[2], parts[3]

    return paths, attributes, float(w), float(h)


# ---------------------------------------------------------------------------
# 6. CSV output
# ---------------------------------------------------------------------------

def save_csv(output_file, sampled_points):
    """
    Write sampled coordinates to a CSV with columns: point_index, x, y.
    """
    with open(output_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["point_index", "x", "y"])
        for i, p in enumerate(sampled_points):
            writer.writerow([i, f"{p.real:.6f}", f"{p.imag:.6f}"])

    print(f"Saved {len(sampled_points)} points -> {output_file}")


# ---------------------------------------------------------------------------
# 7. Matplotlib preview
# ---------------------------------------------------------------------------

def show_preview(paths, attributes, sampled_points):
    """
    Display a matplotlib figure with the original SVG paths drawn as lines
    and the sampled points overlaid as scatter dots.

    SVG coordinates have y increasing downward; we use ax.invert_yaxis()
    to match that convention.
    """
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal")
    ax.set_title("Curvature-weighted samples (red) over original paths (grey)")
    ax.invert_yaxis()  # match SVG coordinate system (y down)

    # Draw original paths by sampling them densely
    for path, attr in zip(paths, attributes):
        t_vals = np.linspace(0, 1, 1000)
        pts = np.array([path.point(t) for t in t_vals])
        color = attr.get("stroke", "#888888")
        if color in ("none", ""):
            color = "#888888"
        ax.plot(pts.real, pts.imag, color=color, linewidth=1, zorder=1)

    # Overlay sampled points
    ax.scatter(
        sampled_points.real, sampled_points.imag,
        s=12, color="#e8503a", zorder=2, linewidths=0,
    )

    # Legend
    path_patch  = mpatches.Patch(color="#888888", label="Original paths")
    point_patch = mpatches.Patch(color="#e8503a", label=f"Sampled points ({len(sampled_points)})")
    ax.legend(handles=[path_patch, point_patch], loc="best")

    ax.set_xlabel("x (SVG units)")
    ax.set_ylabel("y (SVG units)")
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# 8. Main entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Discretise SVG outlines with curvature-weighted point sampling."
    )
    parser.add_argument("input_svg", type=Path, help="Input SVG file.")
    parser.add_argument(
        "-n", "--n-points",
        type=int,
        default=200,
        help="Number of sample points. Default: 200.",
    )
    parser.add_argument(
        "-o", "--output",
        type=Path,
        default=None,
        help="Output CSV path. Defaults to <input>_sampled.csv.",
    )
    parser.add_argument(
        "--uniform-samples",
        type=int,
        default=4000,
        help="Uniform samples per path used internally for curvature estimation. Default: 4000.",
    )
    parser.add_argument(
        "--epsilon",
        type=float,
        default=1e-3,
        help=(
            "Floor on curvature weight (fraction of max curvature) so straight "
            "sections still receive some points. Default: 1e-3."
        ),
    )
    parser.add_argument(
        "--gap-threshold",
        type=float,
        default=1.0,
        help=(
            "Distance (SVG units) below which a path join is considered connected. "
            "Larger gaps trigger a warning. Default: 1.0."
        ),
    )
    parser.add_argument(
        "--smooth-sigma",
        type=float,
        default=5.0,
        help=(
            "Gaussian smoothing applied to coordinates before curvature estimation "
            "(in samples). Higher = smoother, less noise-sensitive. Default: 5.0."
        ),
    )
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="Skip the matplotlib preview.",
    )
    args = parser.parse_args()

    output = args.output or args.input_svg.with_name(args.input_svg.stem + "_sampled.csv")

    # Load paths
    paths, attributes, canvas_w, canvas_h = load_paths(args.input_svg)
    if not paths:
        raise ValueError("No paths found in the SVG.")

    print(f"Found {len(paths)} path(s) in '{args.input_svg}'.")

    # Reorder paths into a continuous chain
    print("Reordering paths into a continuous chain ...")
    ordered_paths = reorder_paths(paths, gap_threshold=args.gap_threshold)

    # Merge into one global polyline
    print("Merging paths into global polyline ...")
    points, arc_norm = merge_paths_to_polyline(
        ordered_paths, n_uniform_per_path=args.uniform_samples
    )
    print(f"  Global polyline: {len(points)} points.")

    # Sample curvature-weighted points from the global polyline
    print(f"Sampling {args.n_points} curvature-weighted points ...")
    sampled_points = curvature_weighted_sample(
        points, arc_norm,
        n_points=args.n_points,
        epsilon=args.epsilon,
        smooth_sigma=args.smooth_sigma,
    )

    # Save CSV
    save_csv(output, sampled_points)

    # Optional preview
    if not args.no_preview:
        show_preview(ordered_paths, attributes, sampled_points)


if __name__ == "__main__":
    main()