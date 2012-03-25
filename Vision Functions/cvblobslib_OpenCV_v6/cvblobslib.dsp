# Microsoft Developer Studio Project File - Name="cvblobslib" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=cvblobslib - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "cvblobslib.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "cvblobslib.mak" CFG="cvblobslib - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "cvblobslib - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE "cvblobslib - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "cvblobslib - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 2
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /MD /W3 /GX /I "c:\Archivos de programa\cv\include" /I "c:\Archivos de programa\OpenCV\cv\include" /I "c:\Archivos de programa\OpenCV\cxcore\include\\" /I "c:\Archivos de programa\OpenCV\OtherLibs\HighGUI\\" /I "c:\Archivos de programa\OpenCV\cv\include\\" /I "..\inspecta\matrix" /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /D "_AFXDLL" /YX /FD /fast /Qopenmp /G7 /QaxW /Qvec_report3 /c
# ADD BASE RSC /l 0xc0a /d "NDEBUG"
# ADD RSC /l 0xc0a /d "NDEBUG" /d "_AFXDLL"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo
# Begin Special Build Tool
SOURCE="$(InputPath)"
PostBuild_Cmds=mkdir lib	echo copiant Release\cvblobslib.lib a lib\cvblobslib.lib	copy Release\cvblobslib.lib lib\cvblobslib.lib
# End Special Build Tool

!ELSEIF  "$(CFG)" == "cvblobslib - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 2
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /YX /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GX /ZI /Od /I "c:\Archivos de programa\OpenCV\cv\include" /I "c:\Archivos de programa\OpenCV\cxcore\include\\" /I "c:\Archivos de programa\OpenCV\OtherLibs\HighGUI\\" /I "c:\Archivos de programa\OpenCV\cv\include\\" /I "..\inspecta\matrix" /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /D "_AFXDLL" /FR /YX /FD /GZ /Qopenmp /c
# ADD BASE RSC /l 0xc0a /d "_DEBUG"
# ADD RSC /l 0xc0a /d "_DEBUG" /d "_AFXDLL"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo
# Begin Special Build Tool
SOURCE="$(InputPath)"
PostBuild_Cmds=mkdir lib	echo copiant Debug\cvblobslib.lib a lib\cvblobslib.lib	copy Debug\cvblobslib.lib lib\cvblobslib.lib
# End Special Build Tool

!ENDIF 

# Begin Target

# Name "cvblobslib - Win32 Release"
# Name "cvblobslib - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=.\blob.cpp
# End Source File
# Begin Source File

SOURCE=.\BlobExtraction.cpp
# End Source File
# Begin Source File

SOURCE=.\BlobResult.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=.\blob.h
# End Source File
# Begin Source File

SOURCE=.\BlobExtraction.h
# End Source File
# Begin Source File

SOURCE=.\BlobResult.h
# End Source File
# End Group
# End Target
# End Project
