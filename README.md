# 2DShapeRenderingPantograph
Code repository for a pantograph that renders 2D shape outline with force and texture feedback.

For further information on the project, visit https://charm.stanford.edu/ME327/2026-Group10#sME327.2026-Group10_6

## Directories
* /main/: contains ImageProcessing.py which processes svgs of shape outlines into text files with arduino array code and csv files for rendering shapes in Processing. In /main/app/ you'll find the app.pde that runs the Processing UI.
* /pantograph_oneboard_final/pantograph_oneboard_final.ino is the final code to run on the main arduino board. The other arduino board on the pantograph should have a blank .ino file uploaded to it. It is only used for sensing.
