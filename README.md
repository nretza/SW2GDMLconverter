# SW2GDMLconverter
Program that converts a CAD design in SolidWorks to GDML format for importation into Geant4. Uses the SolidwWorks API. The model to be converted must be a SolidWorks assembly with all the parts files present.

The executable runs on Windows 7 and requires SolidWorks 2018. The template.gdml file must exist in the current directory when the
converter is run. It is best to run it at a command prompt in order to see any error messages. The output from the converter is
written to a file called design.gdml.
