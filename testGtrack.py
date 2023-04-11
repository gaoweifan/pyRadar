import cppyy
from cppyy import gbl
from cppyy import ll
import numpy as np
from glob import glob

def gtrack_cppyy_init(gtrackRootPath="gtrack",use3D=True):
    # set include path
    cppyy.add_include_path(gtrackRootPath)
    cppyy.add_include_path(gtrackRootPath+"/include/")

    # set 2D/3D macro
    if use3D:
        cppyy.cppdef("#define GTRACK_3D")
        print("GTRACK_3D")
    else:
        cppyy.cppdef("#define GTRACK_2D")
        print("GTRACK_2D")

    print("loading C files")
    # load header file
    cppyy.include(gtrackRootPath+"/gtrack.h")
    # load c files
    for cFile in sorted(glob(gtrackRootPath+"/src/*.c")):
        # print(cFile)
        cppyy.c_include(cFile)
    print("C files loaded")

def gtrack_cppyy_create(use3D=True):
    # instantiation
    config=gbl.GTRACK_moduleConfig()
    advParams=gbl.GTRACK_advancedParameters()

    # Mandatory Configuration Parameters
    config.maxNumPoints = 250
    config.maxNumTracks = 20
    if use3D:
        config.stateVectorType = gbl.GTRACK_STATE_VECTORS_3DA # Track three dimensions with acceleration 
    else:
        config.stateVectorType = gbl.GTRACK_STATE_VECTORS_2DA # Track two dimensions with acceleration 
    config.initialRadialVelocity = 0 # Expected target radial velocity at the moment of detection, m/s
    config.maxAcceleration=np.array([# Maximum targets acceleration in 
                                0.1, # lateral direction
                                0.1, # longitudinal direction
                                0.1  # vertical direction. For 2D options, the vertical component is ignored
                            ],dtype=np.float32)
    config.verbose = gbl.GTRACK_VERBOSE_MAXIMUM
    
    ## This shall match sensor chirp configuration
    config.deltaT = 0.05                    # 50ms per frame
    config.maxRadialVelocity = 5.29         # Radial velocity from sensor is limited to +/- maxURV (in m/s)
    config.radialVelocityResolution = 0.083 # Radial velocity resolution (in m/s)
    
    # Advanced parameters
    ## Scenery Parameters
    sceneryParams = gbl.GTRACK_sceneryParams()
    sceneryParams.sensorPosition=gbl.GTRACK_sensorPosition(0,0,0) # sensor position, (X,Y,Z), is in cartesian space relative to the [3-dimentional] world.
    sceneryParams.sensorOrientation=gbl.GTRACK_sensorOrientation(0,0)# sensor orientation, boresight (azumuthal,elevation) tilt, negative left/up, positive right/down, in degrees

    sceneryParams.numBoundaryBoxes=0   # Number of scene boundary boxes. If defined (numBoundaryBoxes > 0), only points within the boundary box(s) can be associated with tracks
    sceneryParams.boundaryBox[0].x1=-4 # Left boundary, m
    sceneryParams.boundaryBox[0].x2=4  # Right boundary, m
    sceneryParams.boundaryBox[0].y1=0.5# Near boundary, m
    sceneryParams.boundaryBox[0].y2=7.5# Far boundary, m
    sceneryParams.boundaryBox[0].z1=0  # Bottom boundary, m
    sceneryParams.boundaryBox[0].z2=3  # Top boundary, m
    sceneryParams.boundaryBox[1].x1=0  # Left boundary, m
    sceneryParams.boundaryBox[1].x2=0  # Right boundary, m
    sceneryParams.boundaryBox[1].y1=0  # Near boundary, m
    sceneryParams.boundaryBox[1].y2=0  # Far boundary, m
    sceneryParams.boundaryBox[1].z1=0  # Bottom boundary, m
    sceneryParams.boundaryBox[1].z2=0  # Top boundary, m

    sceneryParams.numStaticBoxes=0   # Number of scene static boxes. If defined (numStaticBoxes > 0), only targets within the static box(s) can persist as static
    sceneryParams.staticBox[0].x1=-3 # Left boundary, m
    sceneryParams.staticBox[0].x2=3  # Right boundary, m
    sceneryParams.staticBox[0].y1=2  # Near boundary, m
    sceneryParams.staticBox[0].y2=6  # Far boundary, m
    sceneryParams.staticBox[0].z1=0.5# Bottom boundary, m
    sceneryParams.staticBox[0].z2=2.5# Top boundary, m
    sceneryParams.staticBox[1].x1=0  # Left boundary, m
    sceneryParams.staticBox[1].x2=0  # Right boundary, m
    sceneryParams.staticBox[1].y1=0  # Near boundary, m
    sceneryParams.staticBox[1].y2=0  # Far boundary, m
    sceneryParams.staticBox[1].z1=0  # Bottom boundary, m
    sceneryParams.staticBox[1].z2=0  # Top boundary, m
    
    ## Allocation Parameters
    allocationParams = gbl.GTRACK_allocationParams(60, 200, 0.1, 5, 1, 2)#60 in clear, 200 obscured SNRs, 0.1m/s minimal velocity, 5 points, 1.5m in distance, 2m/s in velocity
    ## State Transition Parameters
    stateParams = gbl.GTRACK_stateParams(10, 5, 50, 100, 5)#det2act, det2free, act2free, stat2free, exit2free
    ## Gating Parameters
    gatingParams = gbl.GTRACK_gatingParams()
    gatingParams.gain=3 # Gating constant gain 3x
    gatingParams.limitsArray=np.array([1.5,1.5,2,0],dtype=np.float32) # Limits are set to 1.5m in depth, width, 2m (if applicable) in height and no limits in doppler
    ## Presence Parameters
    presenceParams = gbl.GTRACK_presenceParams()
    presenceParams.pointsThre=0 # occupancy threshold, number of points. Setting pointsThre to 0 disables presence detection
    
    # load params into config
    advParams.allocationParams = allocationParams
    advParams.gatingParams = gatingParams
    advParams.stateParams = stateParams
    advParams.sceneryParams = sceneryParams
    advParams.presenceParams = presenceParams
    config.advParams = advParams
    
    # init point and target array
    cppyy.cppdef(r"""
        GTRACK_measurementPoint pointCloud[GTRACK_NUM_POINTS_MAX];
        GTRACK_targetDesc targetDescr[GTRACK_NUM_TRACKS_MAX];
    """)

    print("test gtrack_create")
    errno=np.array(0,dtype=np.int32)
    hTrackModule = gbl.gtrack_create(config,errno)
    if errno!=0:
        print("[gtrack_create]errno:",errno)
        return cppyy.nullptr
    hTrackModule = cppyy.bind_object(hTrackModule, 'GtrackModuleInstance')
    print("hTrackModule:",hTrackModule)
    return hTrackModule

def gtrack_cppyy_step(hTrackModule,use3D=True):
    pointCloud=gbl.pointCloud# Pointer to an array of input measurments. Each measurement has range/angle/radial velocity information
    if use3D:
        dim=4
    else:
        dim=3
    pointCloud[0].array=np.array(	[3.83,-0.12,0.28,-0.55],dtype=np.float32)[:dim];    pointCloud[0].SNR=15.32
    pointCloud[1].array=np.array(	[3.83,-0.10,0.28,-0.55],dtype=np.float32)[:dim];    pointCloud[1].SNR=15.92
    pointCloud[2].array=np.array(	[3.78,-0.09,0.23,-0.99],dtype=np.float32)[:dim];    pointCloud[2].SNR=15.21
    pointCloud[3].array=np.array(	[3.83,-0.09,0.28,-0.55],dtype=np.float32)[:dim];    pointCloud[3].SNR=16.55
    pointCloud[4].array=np.array(	[3.78,-0.07,0.23,-0.99],dtype=np.float32)[:dim];    pointCloud[4].SNR=15.85
    pointCloud[5].array=np.array(	[3.83,-0.07,0.28,-0.55],dtype=np.float32)[:dim];    pointCloud[5].SNR=17.10
    pointCloud[6].array=np.array(	[3.78,-0.05,0.23,-0.99],dtype=np.float32)[:dim];    pointCloud[6].SNR=16.49
    pointCloud[7].array=np.array(	[3.83,-0.05,0.28,-0.55],dtype=np.float32)[:dim];    pointCloud[7].SNR=17.44
    pointCloud[8].array=np.array(	[3.88,-0.05,0.33,-0.55],dtype=np.float32)[:dim];    pointCloud[8].SNR=15.57
    pointCloud[9].array=np.array(	[3.93,-0.05,0.21,-0.66],dtype=np.float32)[:dim];    pointCloud[9].SNR=15.31
    pointCloud[10].array=np.array(	[3.98,-0.05,0.21,-0.66],dtype=np.float32)[:dim];    pointCloud[10].SNR=15.91
    pointCloud[11].array=np.array(	[4.03,-0.05,0.19,-0.66],dtype=np.float32)[:dim];    pointCloud[11].SNR=15.36
    pointCloud[12].array=np.array(	[3.78,-0.03,0.23,-0.99],dtype=np.float32)[:dim];    pointCloud[12].SNR=16.87
    pointCloud[13].array=np.array(	[3.83,-0.03,0.28,-0.55],dtype=np.float32)[:dim];    pointCloud[13].SNR=17.45
    pointCloud[14].array=np.array(	[3.88,-0.03,0.33,-0.55],dtype=np.float32)[:dim];    pointCloud[14].SNR=15.94
    pointCloud[15].array=np.array(	[3.93,-0.03,0.21,-0.66],dtype=np.float32)[:dim];    pointCloud[15].SNR=16.52
    pointCloud[16].array=np.array(	[3.98,-0.03,0.21,-0.66],dtype=np.float32)[:dim];    pointCloud[16].SNR=17.33
    pointCloud[17].array=np.array(	[4.08,-0.03,0.19,-0.66],dtype=np.float32)[:dim];    pointCloud[17].SNR=15.57
    pointCloud[18].array=np.array(	[4.13,-0.03,0.21,-0.66],dtype=np.float32)[:dim];    pointCloud[18].SNR=16.51
    pointCloud[19].array=np.array(	[3.78,-0.02,0.23,-0.99],dtype=np.float32)[:dim];    pointCloud[19].SNR=16.70
    pointCloud[20].array=np.array(	[3.83,-0.02,0.28,-0.55],dtype=np.float32)[:dim];    pointCloud[20].SNR=17.20
    pointCloud[21].array=np.array(	[3.88,-0.02,0.33,-0.55],dtype=np.float32)[:dim];    pointCloud[21].SNR=16.00
    pointCloud[22].array=np.array(	[3.93,-0.02,0.21,-0.66],dtype=np.float32)[:dim];    pointCloud[22].SNR=18.22
    pointCloud[23].array=np.array(	[3.98,-0.02,0.21,-0.66],dtype=np.float32)[:dim];    pointCloud[23].SNR=18.78
    pointCloud[24].array=np.array(	[4.08,-0.02,0.19,-0.66],dtype=np.float32)[:dim];    pointCloud[24].SNR=15.83
    pointCloud[25].array=np.array(	[4.13,-0.02,0.21,-0.66],dtype=np.float32)[:dim];    pointCloud[25].SNR=17.79
    pointCloud[26].array=np.array(	[3.78,0.00,0.23,-0.99 ],dtype=np.float32)[:dim];    pointCloud[26].SNR=16.12
    pointCloud[27].array=np.array(	[3.83,0.00,0.28,-0.55 ],dtype=np.float32)[:dim];    pointCloud[27].SNR=16.83
    pointCloud[28].array=np.array(	[3.88,0.00,0.33,-0.55 ],dtype=np.float32)[:dim];    pointCloud[28].SNR=15.70
    pointCloud[29].array=np.array(	[3.93,0.00,0.21,-0.66 ],dtype=np.float32)[:dim];    pointCloud[29].SNR=18.46
    pointCloud[30].array=np.array(	[3.98,0.00,0.21,-0.66 ],dtype=np.float32)[:dim];    pointCloud[30].SNR=17.56
    pointCloud[31].array=np.array(	[4.13,0.00,0.21,-0.66 ],dtype=np.float32)[:dim];    pointCloud[31].SNR=16.73
    pointCloud[32].array=np.array(	[4.28,0.00,0.10,-6.51 ],dtype=np.float32)[:dim];    pointCloud[32].SNR=15.39
    pointCloud[33].array=np.array(	[3.78,0.02,0.23,-0.99 ],dtype=np.float32)[:dim];    pointCloud[33].SNR=15.46
    pointCloud[34].array=np.array(	[3.83,0.02,0.28,-0.55 ],dtype=np.float32)[:dim];    pointCloud[34].SNR=16.42
    pointCloud[35].array=np.array(	[3.88,0.02,0.33,-0.55 ],dtype=np.float32)[:dim];    pointCloud[35].SNR=15.21
    pointCloud[36].array=np.array(	[3.93,0.02,0.21,-0.66 ],dtype=np.float32)[:dim];    pointCloud[36].SNR=16.76
    pointCloud[37].array=np.array(	[3.98,0.02,0.21,-0.66 ],dtype=np.float32)[:dim];    pointCloud[37].SNR=16.03
    pointCloud[38].array=np.array(	[4.13,0.02,0.21,-0.66 ],dtype=np.float32)[:dim];    pointCloud[38].SNR=15.29
    pointCloud[39].array=np.array(	[3.83,0.03,0.28,-0.55 ],dtype=np.float32)[:dim];    pointCloud[39].SNR=15.98
    pointCloud[40].array=np.array(	[3.93,0.03,0.21,-0.66 ],dtype=np.float32)[:dim];    pointCloud[40].SNR=15.51
    pointCloud[41].array=np.array(	[3.83,0.05,0.28,-0.55 ],dtype=np.float32)[:dim];    pointCloud[41].SNR=15.53
    pointCloud[42].array=np.array(	[3.83,0.07,0.28,-0.55 ],dtype=np.float32)[:dim];    pointCloud[42].SNR=15.08
    mNum=43 # Number of input measurements
    targetDescr=gbl.targetDescr# Pointer to an array of GTRACK_targetDesc. This function populates the descritions for each of the tracked target 
    tNum=np.zeros(1,dtype=np.uint16)# Function returns a number of populated target descriptos 
    mIndex=np.zeros(((mNum-1)>>3)+1,dtype=np.uint8)#This function populates target indices, indicating which tracking ID was assigned to each measurment.
    uIndex=np.zeros(mNum,dtype=np.uint8)#This function populates the bit array. The unique-ness of measurement N is represented by a bit = (N & 0xFF) in (N-1)>>3 byte.
    presence=np.zeros(1,dtype=np.uint8)#Pointer to boolean presence indication.
    gbl.gtrack_step(hTrackModule, pointCloud[0], cppyy.nullptr, mNum, targetDescr[0], tNum, mIndex, uIndex, presence, cppyy.nullptr)
    return targetDescr,tNum[0],mIndex,uIndex,presence[0]

def gtrack_cppyy_delete(hTrackModule):
    gbl.gtrack_delete(hTrackModule)


use3D=False
gtrack_cppyy_init(gtrackRootPath="gtrack",use3D=use3D)
hTrackModule=gtrack_cppyy_create(use3D)
if hTrackModule is not cppyy.nullptr:
    targetDescr,tNum,mIndex,uIndex,presence=gtrack_cppyy_step(hTrackModule,use3D)
    print(tNum)
    for i in range(tNum):
        print(targetDescr[i])
    gtrack_cppyy_delete(hTrackModule)