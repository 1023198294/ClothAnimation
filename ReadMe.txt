******************************************************************************
------------------------------
---How to setup for your OS---
------------------------------

----------------------
For windows, MSVC 2019
----------------------
 

Just go into a3_msvc2019 and code there.

Sample executable in a3_msvc2019 -> a3soln.exe


----------------------
For windows, MSVC 2015
----------------------
 

Just go into a3_msvc2015 and code there.

Please do not upgrade to higher platform when using higher version Visual Studio.

Sample executable in a3_msvc2015 -> a3soln.exe

---------
For Linux
---------

Just go into a3_linux and run "make" there for compilation.  

Sample executable in a3_linux -> a3soln

---------
For Mac
---------

Just go into a3_mac and run "make" there for compilation.  

Sample executable in a3_mac -> a3soln

***************************************************************************

a3_linux.zip is tested in Linux OS.
a3_mac.zip is tested in Mac OS.
a3_msvc2015.zip is tested in VS 2015 (MSVC 14.0) (_MSC_VER=1900)
a3_msvc2019.zip is tested in VS 2019 (MSVC 14.2) (_MSC_VER=1923) 


***************************************************************************
Q&A:

Executable Errors:

- Run executable error: "command not found" (Linux, Mac):
  Please make a3soln or a3 a command in terminal: "chmod +x ./a3soln" or "chmod 755 ./a3soln"
  Or (linux) change file "Properties" (right click) > "Permissions"> enable "Allowing executing file as program".

- Error "Permissin denied":
  Please run file as administrator or root user.

- (Mac) Error "can't be open because it is from an unidentified developer":
  Right click -> open.
  Now you can try run the executable in terminal.