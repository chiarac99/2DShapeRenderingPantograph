// UI TOOLS
// ---------
// File for all functions and classes that display and operate buttons


// --------
// classes
// --------

// ── Button ───────────────────────────────────────────────
class Button {
  float x, y, w, h;
  String label;
  boolean clicked = false;
  color bg = color(44, 195, 168);
  color fg = color(0);

  Button(float x, float y, float w, float h, String label) {
    this.x = x; this.y = y; this.w = w; this.h = h;
    this.label = label;
  }

  void draw() {
    fill(bg); noStroke();
    rect(x, y, w, h, 6);
    fill(fg); textAlign(CENTER, CENTER);
    text(label, x + w/2, y + h/2);
    textAlign(LEFT, BASELINE); // reset
    
  }

  void reset() { clicked = false; }

  void handleClick(float mx, float my) {
    if (mx > x && mx < x+w && my > y && my < y+h)
      clicked = true;
  }

  boolean isClicked() { return clicked; }
  void setColors(color bg, color fg) { this.bg = bg; this.fg = fg; }
}


// ── Dropdown ─────────────────────────────────────────────
class Dropdown {
  float x, y, w, h;
  String[] options;
  int[] ids;  // parallel array of ids
  int selectedIndex = 0;
  boolean opened = false;
  color bg = color(255);
  color fg = color(0);
  color optionBg = color(230);

  Dropdown(float x, float y, float w, float h, String[] options, int[] ids) {
    this.x = x; this.y = y; this.w = w; this.h = h;
    this.options = options;
    this.ids = ids;
  }
  void draw() {
    fill(bg); noStroke();
    rect(x, y, w, h, 6);
    fill(fg); textAlign(CENTER, CENTER);
    text(selected() + "  ▼", x + w/2, y + h/2);
    if (opened) {
      for (int i = 0; i < options.length; i++) {
        fill(optionBg); noStroke();
        rect(x, y + h + i*h, w, h);
        fill(fg);
        text(options[i], x + w/2, y + h + i*h + h/2);
      }
    }
    textAlign(LEFT, BASELINE);
  }
  void handleClick(float mx, float my) {
    if (mx > x && mx < x+w && my > y && my < y+h) {
      opened = !opened;
      return;
    }
    if (opened) {
      for (int i = 0; i < options.length; i++) {
        if (mx > x && mx < x+w && my > y+h+i*h && my < y+h+(i+1)*h) {
          selectedIndex = i;
          shapeId = ids[i];  // update global on selection
          opened = false;
          return;
        }
      }
      opened = false;
    }
  }
  String selected() { return options[selectedIndex]; }
  int selectedId() { return ids[selectedIndex]; }
  void setColors(color bg, color fg, color optionBg) {
    this.bg = bg; this.fg = fg; this.optionBg = optionBg;
  }
}

// ── TextInput ─────────────────────────────────────────────
class TextInput {
  float x, y, w, h;
  String placeholder;
  String value = "";
  boolean focused = false;
  color bg = color(255);
  color fg = color(0);
  color placeholderColor = color(160);
  color borderActive = color(44, 195, 168);
  color borderInactive = color(180);

  TextInput(float x, float y, float w, float h, String placeholder) {
    this.x = x; this.y = y; this.w = w; this.h = h;
    this.placeholder = placeholder;
  }

  void draw() {
    // border
    stroke(focused ? borderActive : borderInactive);
    strokeWeight(2);
    fill(bg);
    rect(x, y, w, h, 6);
    noStroke();

    // text or placeholder
    textAlign(LEFT, CENTER);
    if (value.length() == 0 && !focused) {
      fill(placeholderColor);
      text(placeholder, x + 10, y + h/2);
    } else {
      fill(fg);
      String display = value + (focused && frameCount % 60 < 30 ? "|" : "");
      text(display, x + 10, y + h/2);
    }

    textAlign(LEFT, BASELINE); // reset
    strokeWeight(1);           // reset
  }

  void handleClick(float mx, float my) {
    focused = (mx > x && mx < x+w && my > y && my < y+h);
  }

  void handleKey() {
    if (!focused) return;
    if (key == BACKSPACE) {
      if (value.length() > 0)
        value = value.substring(0, value.length() - 1);
    } else if (key == ENTER || key == RETURN) {
      focused = false;
    } else if (key != CODED && key != TAB) {
      value += key;
    }
  }

  String getValue() { return value; }
  void clear() { value = ""; }
  void setColors(color bg, color fg, color border) {
    this.bg = bg; this.fg = fg; this.borderActive = border;
  }
}
