Basketball hoop model - cleaned for use in Webots

Files:
- hoop.obj: model geometry
- hoop.mtl: material file, renamed to avoid non-ASCII path issues
- hoop_texture.png: diffuse texture

The source archive referenced a normal-map image that was not included. The material
has therefore been simplified to use the included diffuse texture only.

Approximate source-model bounds:
- X: 0.320 m
- Y: 1.132 m
- Z: 0.423 m

Suggested Webots import:
1. Copy this folder into your Webots project, for example:
   objects/basketball_hoop/
2. In the Webots Scene Tree, add a Solid node.
3. Under Solid.children, add a Shape node.
4. Set Shape.appearance to PBRAppearance and set its baseColorMap to an
   ImageTexture whose url is "hoop_texture.png".
5. Set Shape.geometry to a CadShape node and set its url to "hoop.obj".
6. Save the world.

Important:
This is a visual mesh only. It does not yet provide a suitable basketball collision
model. Add separate collision geometry for the pole, backboard, and rim before
testing ball impacts. Do not use the whole model's bounding box as the collision
shape, because that would block the ball from passing through the hoop.
