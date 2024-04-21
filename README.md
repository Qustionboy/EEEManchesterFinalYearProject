Enhanced Readability:
Generation of 3D Grape Model
https://github.com/Qustionboy/EEEManchesterFinalYearProject
Overview:
The project is focused on creating 3D grape models and their Portable Gray Map (PGM) file representations.

Installation:
The package requires the Point Cloud Library (PCL) as a dependency, which must be installed prior to using the package:
https://pointclouds.org/downloads/
It is highly recommended that vcpkg be utilized for the installation process.

Follow this guide to install vcpkg on Windows systems:
https://www.studyplan.dev/pro-cpp/vcpkg-windows

To install PCL on a desktop with vcpkg, input the following command in your terminal:
PS> .\vcpkg install pcl

Configuring PCL in Visual Studio involves:
Entering the following commands in the vcpkg directory:
vcpkg integrate project
vcpkg integrate install

Open Visual Studio 2023 and navigate to Tools > NuGet Package Manager > Package Manager Settings.
In the left-hand menu, select Package Sources.
Add vcpkg as a Package Source.

After completing these steps, the program should compile successfully.

Release Versions:
Two release versions of the project are available.
Release_Test is a version that tests the performance of different algorithms.
Release_Final is the definitive version of the project.
The release program can be directly opened by unzipping the file and clicking the .exe file.

Open the output:
Since the PCL installed by vcpkg does not provide a PCL viewer, CloudCompare is recommanded to open the PCD file, which is the software's output.
The following is the official website of CloudCompare:
https://www.cloudcompare.org/

