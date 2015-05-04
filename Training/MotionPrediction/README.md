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
