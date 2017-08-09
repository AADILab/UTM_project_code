"carrie_lib"

This is the version of rebhuhnc/libraries with pre-compiled files.

Source files are included, and anything under development can be re-compiled. To update the object files perform the following steps.

WINDOWS:
1. Open vcc/Libs.sln in Visual Studio
2. Select desired build mode (Debug/Release x84/x64)
3. Clean and build

LINUX:
1. chmod +x carrie_build.sh
2. ./carrie_make.sh

Updates:
- There is no 'domain' lib. This is still under development and should be recompiled each time.
- Doxygen has been removed from source control. Doxygen is still supported but must be run externally
