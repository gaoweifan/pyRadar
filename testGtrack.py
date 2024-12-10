import cppyy
from cppyy import gbl
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
    # set debug log macro
    cppyy.cppdef("#define GTRACK_LOG_ENABLED")

    print("loading C files")
    # load header file
    cppyy.include(gtrackRootPath+"/gtrack.h")
    # load c files
    for cFile in sorted(glob(gtrackRootPath+"/src/*.c")):
        # print(cFile)
        cppyy.c_include(cFile)
    print("C files loaded")

    # Ensure global variables are resolved
    (gbl.pInit, gbl.spreadMin)

def gtrack_cppyy_create(use3D=True):
    # instantiation
    config=gbl.GTRACK_moduleConfig()
    advParams=gbl.GTRACK_advancedParameters()

    # Mandatory Configuration Parameters
    config.maxNumPoints = 500
    config.maxNumTracks = 50
    if use3D:
        config.stateVectorType = gbl.GTRACK_STATE_VECTORS_3DA # Track three dimensions with acceleration 
    else:
        config.stateVectorType = gbl.GTRACK_STATE_VECTORS_2DA # Track two dimensions with acceleration 
    config.initialRadialVelocity = 0 # Expected target radial velocity at the moment of detection, m/s
    config.maxAcceleration=np.array([# Maximum targets acceleration in 
                                  1, # lateral direction (m/s2)
                                  1, # longitudinal direction (m/s2)
                                  1  # vertical direction (m/s2). For 2D options, the vertical component is ignored
                            ],dtype=np.float32)
    config.verbose = gbl.GTRACK_VERBOSE_MAXIMUM
    # config.verbose = gbl.GTRACK_VERBOSE_DEBUG
    # config.verbose = gbl.GTRACK_VERBOSE_NONE
    
    ## This shall match sensor chirp configuration
    config.deltaT = 0.05                    # 50ms per frame
    config.maxRadialVelocity = 5.29         # Radial velocity from sensor is limited to +/- maxURV (in m/s)
    config.radialVelocityResolution = 0.083 # Radial velocity resolution (in m/s)
    
    # Advanced parameters
    ## Scenery Parameters
    sceneryParams = gbl.GTRACK_sceneryParams()
    sceneryParams.sensorPosition=gbl.GTRACK_sensorPosition(0,0,0) # sensor position, (X,Y,Z), is in cartesian space relative to the [3-dimentional] world.
    sceneryParams.sensorOrientation=gbl.GTRACK_sensorOrientation(0,0)# sensor orientation, boresight (azumuthal,elevation) tilt, negative left/up, positive right/down, in degrees

    sceneryParams.numBoundaryBoxes=1   # Number of scene boundary boxes. If defined (numBoundaryBoxes > 0), only points within the boundary box(s) can be associated with tracks
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

    sceneryParams.numStaticBoxes=1   # Number of scene static boxes. If defined (numStaticBoxes > 0), only targets within the static box(s) can persist as static
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

def gtrack_cppyy_step(hTrackModule,pointCloudList,use3D=True):
    mNum = len(pointCloudList) # Number of input measurements
    maxNum = cppyy.macro("GTRACK_NUM_POINTS_MAX")
    if(mNum > maxNum):
        raise ValueError(f"point num: {mNum}, exceed maximum supported num: {maxNum}")
    
    dim = 4 if use3D else 3
    pointCloud = gbl.pointCloud # Pointer to an array of input measurments. Each measurement has range/angle/radial velocity information
    for idx, pointInfo in enumerate(pointCloudList):
        pointCloud[idx].array = np.array(pointInfo[0],dtype=np.float32)[:dim]
        pointCloud[idx].snr = pointInfo[1]
    
    var = cppyy.nullptr # Pointer to an array of input measurment variances. Shall be set to NULL if variances are unknown
    targetDescr = gbl.targetDescr # Pointer to an array of GTRACK_targetDesc. gtrack_step populates the descritions for each of the tracked target 
    tNum = np.zeros(1,dtype=np.uint16) # gtrack_step returns a number of populated target descriptos 
    mIndex = np.zeros(((mNum-1)>>3)+1,dtype=np.uint8) # gtrack_step populates target indices, indicating which tracking ID was assigned to each measurment.
    uIndex = np.zeros(mNum,dtype=np.uint8) # gtrack_step populates the bit array. The unique-ness of measurement N is represented by a bit = (N & 0xFF) in (N-1)>>3 byte.
    presence = np.zeros(1,dtype=np.uint8) # Pointer to boolean presence indication.
    bench = cppyy.nullptr # Pointer to an array of benchmarking results. Shall be set to NULL when benchmarking isn't required.

    gbl.gtrack_step(hTrackModule, pointCloud[0], var, mNum, targetDescr[0], tNum, mIndex, uIndex, presence, bench)
    
    return targetDescr,tNum[0],mIndex,uIndex,presence[0]

def gtrack_cppyy_delete(hTrackModule):
    gbl.gtrack_delete(hTrackModule)


test3DPointCloudDataList = [
#   ([Range,Azimuth,Elevation,velocity], snr )
#       m     rad      rad       m/s
    ([3.83,  -0.12,   0.28,    -0.55],  15.32),
    ([3.83,  -0.10,   0.28,    -0.55],  15.92),
    ([3.78,  -0.09,   0.23,    -0.99],  15.21),
    ([3.83,  -0.09,   0.28,    -0.55],  16.55),
    ([3.78,  -0.07,   0.23,    -0.99],  15.85),
    ([3.83,  -0.07,   0.28,    -0.55],  17.10),
    ([3.78,  -0.05,   0.23,    -0.99],  16.49),
    ([3.83,  -0.05,   0.28,    -0.55],  17.44),
    ([3.88,  -0.05,   0.33,    -0.55],  15.57),
    ([3.93,  -0.05,   0.21,    -0.66],  15.31),
    ([3.98,  -0.05,   0.21,    -0.66],  15.91),
    ([4.03,  -0.05,   0.19,    -0.66],  15.36),
    ([3.78,  -0.03,   0.23,    -0.99],  16.87),
    ([3.83,  -0.03,   0.28,    -0.55],  17.45),
    ([3.88,  -0.03,   0.33,    -0.55],  15.94),
    ([3.93,  -0.03,   0.21,    -0.66],  16.52),
    ([3.98,  -0.03,   0.21,    -0.66],  17.33),
    ([4.08,  -0.03,   0.19,    -0.66],  15.57),
    ([4.13,  -0.03,   0.21,    -0.66],  16.51),
    ([3.78,  -0.02,   0.23,    -0.99],  16.70),
    ([3.83,  -0.02,   0.28,    -0.55],  17.20),
    ([3.88,  -0.02,   0.33,    -0.55],  16.00),
    ([3.93,  -0.02,   0.21,    -0.66],  18.22),
    ([3.98,  -0.02,   0.21,    -0.66],  18.78),
    ([4.08,  -0.02,   0.19,    -0.66],  15.83),
    ([4.13,  -0.02,   0.21,    -0.66],  17.79),
    ([3.78,   0.00,   0.23,    -0.99],  16.12),
    ([3.83,   0.00,   0.28,    -0.55],  16.83),
    ([3.88,   0.00,   0.33,    -0.55],  15.70),
    ([3.93,   0.00,   0.21,    -0.66],  18.46),
    ([3.98,   0.00,   0.21,    -0.66],  17.56),
    ([4.13,   0.00,   0.21,    -0.66],  16.73),
    ([4.28,   0.00,   0.10,    -6.51],  15.39),
    ([3.78,   0.02,   0.23,    -0.99],  15.46),
    ([3.83,   0.02,   0.28,    -0.55],  16.42),
    ([3.88,   0.02,   0.33,    -0.55],  15.21),
    ([3.93,   0.02,   0.21,    -0.66],  16.76),
    ([3.98,   0.02,   0.21,    -0.66],  16.03),
    ([4.13,   0.02,   0.21,    -0.66],  15.29),
    ([3.83,   0.03,   0.28,    -0.55],  15.98),
    ([3.93,   0.03,   0.21,    -0.66],  15.51),
    ([3.83,   0.05,   0.28,    -0.55],  15.53),
    ([3.83,   0.07,   0.28,    -0.55],  15.08)
]

if __name__ == '__main__':
    use3D=True
    gtrack_cppyy_init(gtrackRootPath="gtrack",use3D=use3D)
    hTrackModule=gtrack_cppyy_create(use3D)
    if hTrackModule is not cppyy.nullptr:
        targetDescr,tNum,mIndex,uIndex,presence=gtrack_cppyy_step(hTrackModule,test3DPointCloudDataList,use3D)
        print(f"Detected Target Number: {tNum}")
        for i in range(tNum):
            print(f"  Target index: {i}")
            stateVector=np.frombuffer(targetDescr[i].S, dtype=np.float32).reshape((-1,3))
            centroid=np.frombuffer(targetDescr[i].uCenter, dtype=np.float32)
            print(f"    Target State vector:\n{stateVector}") # pos(X,Y,Z),vel(X,Y,Z),acc(X,Y,Z)
            print(f"    Measurement Centroid:\n{centroid}") # Range(m),Azimuth(rad),Elevation(rad),Radial velocity(m/s)
        gtrack_cppyy_delete(hTrackModule)