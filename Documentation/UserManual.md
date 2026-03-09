# Library functions documentation

## Get Started

1. `git pull` this repository

2. Open the project in STM32CubeIDE

3. Go to Project -> Properties -> C/C++ General -> Paths and Symbols -> Source Location

    1. Press "Link Folder.."
    2. Name "LoRaMesh"
    3. Check "Link to folder in the file system"
    4. Enter LoRaMesh-Library location

4. Go to Project -> Properties -> C/C++ Build -> Settings -> Tool Settings -> MCU GCC Compiler -> Include paths (-l)

    1. Add a new entry
    2. Press "Workspace..."
    3. Select previously linked folder "LoRaMesh"
    4. Choose "Inc"
    5. Press "OK"

5. Build and check for errors

## Library Functions

### LoRa.c and LoRa.h

Functions

### Mesh.c and Mesh.h

Functions
