GTRACK Code development notes

Code repository
ssh://git@bitbucket.itg.ti.com/mmwave/mmwave_sdk.git

Group tracker development branch name:
feature/MMWSDK-764-group-tracking-development

Steps to get the code

git clone ssh://git@bitbucket.itg.ti.com/mmwave/mmwave_sdk.git
git pull
git checkout feature/MMWSDK-764-group-tracking-development

The directory structure
-----------------------
Code is in /ti/alg/gtrack directory

/build
	/win/msvc 
			Projects to build gtrack libraries to use with either MSVC (gtrackC.liv) or MATLAB (gtrackMex.lib) test environments. 
			Solutions and projects worked for Visual Studio Professional 2012
/docs
			gtrack code includes support for doxigen. External APIs, Unit level APIs, Math and Utility functions are described

/include
			gtrack code internal headers
/src
			C code sources
			Module level functions are in gtrackModuleXXX.c files
			Unit level functions are in gtrackUnitXXX.c

/test
	/vectors
		/urv20 and /urv100 
			Each directory has 10 point cloud files in ascii format. 
			Each file represents one minute of simulated run (1500@40ms frames). 
			Simulations are identical except urv20 simulated unambigiuos velocity of +-20m/s, and urv100 of +/- 100m/s.

	/win/matlab
		/mex 
			Has c source files to use gtrackMex libray. Also has mex_compile.m script to complie. Was used with MATLAB 2016b version.
		/src 
			Has MATLAB source files for MATLAB implementation of gtrack algorithm. Functionaly close to c implementation.
		/tm 
			Has full MATLAB traffic monitoring simulation. tm.m is a main file to run. 
			Code includes multi-lane vehicle arrival model, traffic light simulation, and vehicle motion modeling.
			Point cloud simulation is based on statistical models taken from cloudStatXXX.mat files.
			Group tracking visulization shows history, centroid, gating and association functions.
			Point Cloud used in test vectors, as well as vehicle ground truth data are generated using tm runs with randomState2.mat seeds
		/unit1500
			gtrackTest.m takes Point Clous (10files x 1500frames each) as input, and runs in parallel C and MATLAB implementations. 
			Visualization is available to compare, as well as automated comparison checks.
	/win/msvc
			Has pure C test application. Visual C solution and project. 
			Runs Point cloud (10files x 1500frames each) input, outputs results into a concole.
