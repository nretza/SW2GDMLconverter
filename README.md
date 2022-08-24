# SW2GDMLconverter

This is a fork from https://github.com/cvuosalo/SW2GDMLconverter. The program was compiled in vs code against the Solidworks 2022 API.

To compile, install [visual studio build tools](https://aka.ms/vs/17/release/vs_BuildTools.exe) with Windows API. Run vs code through the vs developer command promt, so all path variables are set correctly. Compile through the vs code build task.

## original readme:
Program that converts a CAD design in SolidWorks to GDML format for importation into Geant4. Uses the SolidwWorks API. The model to be converted must be a SolidWorks assembly with all the parts files present.

The executable runs on Windows 10 and requires SolidWorks 2018. The template.gdml file must exist in the current directory when the
converter is run. It is best to run it at a command prompt in order to see any error messages. The output from the converter is
written to a file called design.gdml.
