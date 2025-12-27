# FRC2026

### Tech stack:
- Java + lombok(kotlin cope)
- AdvantageKit
- MapleSim
- AdvantageScope
- ChoreoLib
- Phoenix6, REVLib, PhotonLib, etc.

### Important Folders
- /src/main - RIO robot code
- /scripts - python utilities, including vision recording and path mirroring
- /advantagescope-assets - Common assets folder for 3d models/layout

### Regenerating Choreo Trajectories
- Open the terminal, then run ```cd C:\Users\Your_Name\AppData\Local\Choreo```.
- Right click the Autos.chor file(in deploy/choreo), click "Copy path/reference", then "Absolute Path" 
- Then, run ```.\choreo-cli.exe --chor [path you just copied] --all-trajectory -g``` in the terminal.