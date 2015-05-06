[https://github.com/ethz-asl/libpointmatcher/blob/master/doc/Configuration.md](https://github.com/ethz-asl/libpointmatcher/blob/master/doc/Configuration.md)

Datapoint Filters -

1. Bounding Box - Not Needed 
2. Maximum Density Filter - Not Needed 
3. Maximum Distance Filter - Not Needed 
4. Minimum Distance Filter - Not Needed 
5. Maximum Point Cloud Filter Not Needed 
6. Maximum Quantile on Axis Filter - Not Needed 
7. Random Sampling Filter - Not Needed 
8. Remove NaN Filter - Not Needed 
9. Shadow Point Filter - Not Needed 
10. Voxel Grid Filter - Can try this for a high density cloud (This is better than picking every fourth value). Can reduce Artifacts (Like nose coming out very pointed) 
    1. VoxelGridDataPointsFilter 

vSizeX: 1.0

vSizeY: 1.0

vSizeZ: 1.0

useCentroid: 1

average ExistingDescriptors: 1

      11)  Another experiment, use our sampling method (every fourth value, vs random sampling method)

Augmentation Filter -

1. Observation Direction Filter - 
    1. ObservationDirectionDataPointsFilterx:0.0y:0.0z:0.0 

2. Surface Normal Filter - 
    1. SurfaceNormalDataPointsFilterknn:5epsilon:0.0keepNormals: 1keepDesnities: 0keepEigenValues: 0keepEigenVectors: 0keepMatchedIds: 0 

3. Orient Normals Filter 
    1. Needs SurfaceNormalDataPointsFilter and ObservationDirectionDataPointsFilter  
    2. OrientNormalsDataPointsFiltertowardCenter: 1 

4. Sampling Surface Normal Filters - Two Takes  
    1. This generally useful for Planar structures where you expect most normal information to be redundant. We are subsampling and are using face contours where we wont see a lot of redundant normals. 
    2. Or we could use this by providing it with a high density point cloud. This would reduce artifacts and help in accurate matching. 

Matches

1. KDTreeMatcher - Nearest Neighbor Search 
2. KDTreeVarDistMatcher - Approximate Nearest Neighbor Search 

According to this paper  [http://e-collection.library.ethz.ch/eserv/eth:7668/eth-7668-01.pdf](http://e-collection.library.ethz.ch/eserv/eth:7668/eth-7668-01.pdf) approximate nearest neighbors gives a faster match for a performance penalty however performance degradation rate is very low. Check out section 5C of the paper. It uses the same library as we do to do some design space exploration.

outlier Filters

1. TrimmedDistOutlierFilter - One trimmed distance outlier filter is used. The filter ranks the points in the reading cloud by their distance to the reference after a transformation was applied. The top 85% points (those with the smallest distances) are kept. (http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1047997&tag=1) 
    1. TrimmedDistOutlierFilter 

2. MaxDistOutlierFilter - This filter considers as outlier links whose norms are above a fix threshold 
    1. MaxDistOutlierFiltermaxDist : 1 

3. MinDistOutlierFilter - This filter considers as outlier links whose norms are below a threshold 
    1. MinDistOutlierFilterminDist: 1 

4. MedianDistOutlierFilter - This filter considers as outlier links whose norms are above the median link norms times a factor 
    1. MedianDistOutlierFilterfactor: 3 (points farther away factor * median will be considered outliers. ) 

5. VarTrimmedDistOutlierFilter - Hard rejection threshold using quantile and variable ratio (http://www.cs.utah.edu/~jeffp/papers/FICP-3DIM07.pdf) 
    1. VarTrimmedDistOutlierFilterminRatio : 0.05maxRatio : 0.99lambda: 0.95  

Error Minimizer

1. Point To Plane - "force2D", "If set to true(1), the minimization will be force to give a solution in 2D (i.e., on the XY-plane) even with 3D inputs.  http://www.math.zju.edu.cn/cagd/seminar/2007_autumnwinter/2007_autumn_master_liuyu_ref_2.pdf 
    1. PointToPlaneErrorMinimizerforce2D:1 

2. Point to Point Error Minimizer - "A Method for Registration of 3-D Shapes" 

Transformation Checker

1. BoundTransformationChecker - This checker stops the ICP loop with an exception when the transformation values exceed bounds 
    1. BoundTransformationCheckermaxRotationNorm: 1 (rotation bound)maxTranslationNorm: 1 (translation bound) 

2. DifferentialTransformationChecker - This checker stops the ICP loop when the relative motions (i.e. abs(currentIter - lastIter)) of rotation and translation components are below a fix thresholds. This allows to stop the iteration when the point cloud is stabilized. Smoothing can be applied to avoid oscillations, "The Trimmed Iterative Closest Point Algorithm" 
    1. DifferentialTransformationCheckerminDiffRotErr: 0.001 (rotation error threshold)minDifTransErr: 0.001smoothLength: 3 

3. CounterTransformationChecker -  
    1. CounterTransformationCheckermaxIterationCount: 40
