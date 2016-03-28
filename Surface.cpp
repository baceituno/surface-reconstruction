/*********************************************************************/
/* File: surface.cpp                                                 */
/* Last Edition: 27/03/2016, 09:20 PM.                               */
/*********************************************************************/
/* Programmed by:                                                    */
/* Bernardo Aceituno C                                               */
/*********************************************************************/
/*Surface reconstruction module, receives an unordered point cloud   */
/*and estimates its surface through Poisson reconstruction and normal*/
/*estimation.                                                        */
/*********************************************************************/

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/bilateral_upsampling.h>

#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/clean.h>

#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/complex/algorithms/create/ball_pivoting.h>

#include <wrap/io_trimesh/export_ply.h>

#include <help.h>

#include <iostream>
#include <unistd.h>
#include <stdlib.h>

#define ALG_POISSON 1
#define ALG_BALLPIV 2
#define ALG_SPLINE  3

using namespace std;
using namespace pcl;
using namespace vcg;

void printUsage(){
	cout << "\nSurface reconstruction module" << endl;
    cout << "\nUsage:\n$ Surface -i input.pcd -o output.ply -a <reconstruction algorithm> -u <[Optional] process the unfiltered cloud> -d <Poisson depth> -r Ball Pivoting radius -m Ball Pivoting MLS radius -n Ball Pivoting normal estimation radius -c Ball Pivoting clustering\n\n" << endl;
    cout << "Supported algorithms: " << endl;
    cout << '\t' << "*\tPoisson:      \tenter -a Poisson, poisson, p or P." << endl;
    cout << '\t' << "*\tBall Pivoting:\tenter -a Ball_Pivoting, ball_pivoting, bp or BP." << endl;
    cout << '\t' << "*\tB-Spline Aproximation:\tenter -a BS, bs, spline, s." << endl;
}

void PointCloud2Vector3d (PointCloud<PointXYZ>::Ptr cloud, on_nurbs::vector_vec3d &data){
	for (unsigned i = 0; i < cloud->size (); i++){
	  PointXYZ &p = cloud->at(i);
	  if (!pcl_isnan (p.x) && !pcl_isnan (p.y) && !pcl_isnan (p.z))
	    data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
	}
}

int main (int argc, char **argv){
  	//help section
 	if(argc < 7 || string(argv[1]) == "-h"){
    	//Prints the usage instructions
  		printUsage();
    	return 0;
	}

	string inputfile, outputfile;
	//variable for the algorithm
	//1 por poisson, 2 for ball pivoting and 3 for VGC
	int algorithm = 0;

	//apply Voxel Grid to point cloud
	int filter = 1; 

	//counter
	int i;
	int depth = 9;
	int opt = 0;
	int bup = 0;
	//Ball pivoting parameters
	float radius = 0.0f;
	float clustering = 0.05;
	float mls_sr = 1.2;
	float ne_sr  = 0.5;
 
	string argval;

	while ((opt = getopt(argc, argv, "i:o:u:a:d:r:c:m:n:b")) != -1){
		switch(opt){
			case 'i':
				inputfile = string(optarg);
				break;
			case 'o':
				outputfile = string(optarg);
				break;
			case 'u':
	      		cout << "VoxelGrid unabled" << endl;
				filter = 0;
				break;
			case 'a':
				argval = string (optarg);
				if(argval == "p" || argval == "poisson" || argval == "Poisson") algorithm = ALG_POISSON;
	     		else if(argval == "bp" || argval == "BP" || argval == "ball_pivoting" || argval == "Ball_Pivoting") algorithm = ALG_BALLPIV;
	      		else if(argval == "BS" || argval == "bs" || argval == "spline" || argval == "s") algorithm = ALG_SPLINE;
				break;
			case 'd':
				depth = atof(optarg);
				break;
	    	case 'r':
	      		radius = atof(optarg);
	      		break;
	      	case 'c':
				clustering = atof(optarg);
				break;
	    	case 'm':
	      		mls_sr = atof(optarg);
	      		break;
	      	case 'n':
				ne_sr = atof(optarg);
				break;
	    	case 'b':
	      		bup = 1;
	      		break;
			case '?':
				if (optopt == 'i') {
					cout << "Missing mandatory input option" << endl;
				}
				else if (optopt == 'o') {
					cout << "Missing mandatory output option" << endl;
				} 
				else if (optopt == 'a') {
					cout << "Missing mandatory output option" << endl;
				} 
				else {
					cout << "Invalid option received" << endl;
					}
				break;
		}
	}

	//declares the Point Clouds
	PCLPointCloud2::Ptr cloud_2(new PCLPointCloud2 ());
	PCLPointCloud2::Ptr unf_cloud(new PCLPointCloud2 ());
	PointCloud<PointXYZ>::Ptr cloud_aux(new PointCloud<PointXYZ> ());
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ> ());
	PointCloud<PointXYZ>::Ptr cloud_s (new PointCloud<PointXYZ>);
	if(pcl::io::loadPLYFile(inputfile.c_str(), *unf_cloud) == -1)
		pcl::io::loadPCDFile(inputfile.c_str(), *unf_cloud);

	cout << "Point Cloud " << inputfile << " succesfully openned!" << endl;

	//Filter the point cloud through Voxel Grid
	if(filter == 1){ 
	    VoxelGrid<PCLPointCloud2> sor;
	    sor.setInputCloud(unf_cloud);
	    sor.setLeafSize(0.01f, 0.01f, 0.01f);
	    sor.filter(*cloud_2);
	    //converts the cloud format
	    fromPCLPointCloud2(*cloud_2, *cloud_aux);
	    
	    //removes outliears
	    StatisticalOutlierRemoval<PointXYZ> sor2;
	    sor2.setInputCloud (cloud_aux);
	    sor2.setMeanK (50);
	    sor2.setStddevMulThresh (1.0);
	    sor2.filter (*cloud);

	    toPCLPointCloud2(*cloud, *unf_cloud);
	    fromPCLPointCloud2(*unf_cloud,*cloud_s);
	}
	else{
	    //removes outliears
	    fromPCLPointCloud2(*unf_cloud, *cloud);

	    //removes outliers
	    StatisticalOutlierRemoval<PointXYZ> sor2;
	    sor2.setInputCloud (cloud);
	    sor2.setMeanK (50);
	    sor2.setStddevMulThresh (1.0);
	    sor2.filter (*cloud);

	    toPCLPointCloud2(*cloud, *unf_cloud);
	    fromPCLPointCloud2(*unf_cloud,*cloud_s);   	
	}

	if(bup == 1){
	    //declarations
	    PointCloud<PointXYZRGBA>::Ptr cloud_bu(new PointCloud<PointXYZRGBA> ());
	    PointCloud<PointXYZRGBA>::Ptr cloud_abu(new PointCloud<PointXYZRGBA> ());

	    //transforms format
	    copyPointCloud<PointXYZ,PointXYZRGBA>(*cloud, *cloud_abu);

	    //performs bilateral upsampling
	    BilateralUpsampling<PointXYZRGBA, PointXYZRGBA> bu;
	    bu.setInputCloud(cloud_bu);
	    bu.setWindowSize(15);
	    bu.setSigmaColor(15);
	    bu.setSigmaDepth(1.5);
	    bu.process(*cloud_abu);

    	//transforms format
    	copyPointCloud<PointXYZRGBA,PointXYZ>(*cloud_abu, *cloud);
	}

	if(algorithm == ALG_POISSON){
  		cout << "\nRunning Poisson reconstruction...\n" << endl;

  		//Declars the Moving Least Squares
  		MovingLeastSquares<PointXYZ, PointXYZ> mls;
  	    
  		//declares the search method
  		search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);

 	 	//configures the MLS
  		mls.setInputCloud (cloud);
  		mls.setPolynomialFit (true);
  		mls.setSearchMethod (tree);
  		mls.setSearchRadius (0.01);
  		mls.setPolynomialOrder (2);
  		mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
	  	mls.setUpsamplingRadius (0.005);
  		mls.setUpsamplingStepSize (0.003);

	  	//Estimates the normals and smoothes the surface
  		PointCloud<PointXYZ>::Ptr cloud_smoothed(new PointCloud<PointXYZ> ());
  		mls.process(*cloud_smoothed);

	  	NormalEstimationOMP<PointXYZ, pcl::Normal> ne;
  		ne.setNumberOfThreads(8);
  		ne.setInputCloud(cloud_smoothed);
  		ne.setRadiusSearch (0.01);
  		Eigen::Vector4f centroid;
  		compute3DCentroid(*cloud_smoothed, centroid);
  		ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

	    //Estimates normals
   		PointCloud<pcl::Normal>::Ptr cloud_normals(new PointCloud<pcl::Normal> ());
    	ne.compute(*cloud_normals);

	    for(size_t i = 0; i < cloud_normals->size (); ++i){
    		cloud_normals->points[i].normal_x *= -1;
    		cloud_normals->points[i].normal_y *= -1;
	   		cloud_normals->points[i].normal_z *= -1;
    	}

    	PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal> ());
    	concatenateFields(*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);

 	   	//Performs the poisson reconstruction
    	Poisson<PointNormal> poisson;
    	poisson.setDepth(depth);
    	poisson.setInputCloud(cloud_smoothed_normals);
	    PolygonMesh mesh;
    	poisson.reconstruct(mesh);

    	cout << "Reconstruction done!"<< endl;
    	//saves the reconstructed mesh
	    cout << "\nSaving ply file: "  << outputfile << endl;
    	pcl::io::savePLYFile(outputfile.c_str(), mesh);
	}
  
	else if(algorithm == ALG_BALLPIV){

	    cout << "\nBall Pivoting Algorithm:"  << endl;

	    //mesh declaration
	    MyMesh m;

	    cout << "\nCloud Upsampling..."  << endl;
	    //Declars the Moving Least Squares
  		MovingLeastSquares<PointXYZ, PointXYZ> mls;
  	    
  		//declares the search method
  		search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);

 	 	//configures the MLS
  		mls.setInputCloud (cloud);
  		mls.setPolynomialFit (true);
  		mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZ>::VOXEL_GRID_DILATION);
  		mls.setSearchMethod (tree);
  		mls.setSearchRadius (mls_sr);

	  	//Estimates the normals and smoothes the surface
  		PointCloud<PointXYZ>::Ptr cloud_smoothed(new PointCloud<PointXYZ> ());
  		mls.process(*cloud_smoothed);

  		cout << "\nEstimating Normals..."  << endl;
	  	NormalEstimationOMP<PointXYZ, pcl::Normal> ne;
  		ne.setNumberOfThreads(8);
  		ne.setInputCloud(cloud_smoothed);
  		ne.setRadiusSearch (ne_sr);
  		Eigen::Vector4f centroid;
  		compute3DCentroid(*cloud_smoothed, centroid);
  		ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

	    //Estimates normals
   		PointCloud<pcl::Normal>::Ptr cloud_normals(new PointCloud<pcl::Normal> ());
    	ne.compute(*cloud_normals);

	    for(size_t i = 0; i < cloud_normals->size(); ++i){
    		cloud_normals->points[i].normal_x *= -1;
    		cloud_normals->points[i].normal_y *= -1;
	   		cloud_normals->points[i].normal_z *= -1;
    	}

    	PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal> ());
    	concatenateFields(*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);

  		cout << "\nVertex conversion..."  << endl;
	    //Point cloud conversion
	    int vertCount = cloud_smoothed_normals->width*cloud_smoothed_normals->height;
	    vcg::tri::Allocator<MyMesh>::AddVertices(m, vertCount);
	    for(i = 0 ; i < vertCount ; i++){
	      m.vert[i].P() = vcg::Point3f(cloud_smoothed_normals->points[i].x,cloud_smoothed_normals->points[i].y,cloud_smoothed_normals->points[i].z);
	    }

	    //Running BPA
	    cout << "\nRunning BPA..."  << endl;
	    vcg::tri::UpdateBounding<MyMesh>::Box(m);
	    vcg::tri::UpdateNormal<MyMesh>::PerFace(m);
	    tri::BallPivoting<MyMesh> pivot(m, radius, clustering);
	    //Finisiong
	    pivot.BuildMesh();
	    cout << "\nReconstruction done!"<< endl;

	    //orienting normals
	    vcg::tri::UpdateBounding<MyMesh>::Box(m);
	    vcg::tri::UpdateNormal<MyMesh>::PerFace(m);
	    vcg::tri::UpdateNormal<MyMesh>::PerVertexNormalizedPerFace(m);

	    vcg::tri::UpdateTopology<MyMesh>::FaceFace(m);
	    bool oriented, orientable;
	    vcg::tri::Clean<MyMesh>::OrientCoherentlyMesh(m, oriented,orientable);
	    vcg::tri::UpdateNormal<MyMesh>::PerVertexNormalizedPerFace(m);
	    vcg::tri::UpdateNormal<MyMesh>::PerVertexFromCurrentFaceNormal(m);

	    //Exporting ply output file
	    cout << "\nSaving ply file: "  << outputfile << endl;
	    vcg::tri::io::PlyInfo pi;
	    vcg::tri::io::ExporterPLY<MyMesh>::Save(m,outputfile.c_str(),pi.mask);
  	}
	
	else if(algorithm == ALG_SPLINE){
		cout << "Not yet implemented" << endl;
	  	/*on_nurbs::NurbsDataSurface data;
	  	PointCloud2Vector3d (cloud_s, data.interior);

	  	unsigned order (2);
		unsigned refinement (4);
		unsigned iterations (4);
		unsigned mesh_resolution (128);

		on_nurbs::FittingSurface::Parameter params;
		params.interior_smoothness = 0.2;
		params.interior_weight = 1.0;
		params.boundary_smoothness = 0.2;
		params.boundary_weight = 0.0;

		cout << "\nRunning B-Spline reconstruction!"<< endl;

		ON_NurbsSurface nurbs = on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data);
		on_nurbs::FittingSurface fit (&data, nurbs);
		PolygonMesh mesh;
	  	PointCloud<PointXYZ>::Ptr mesh_cloud (new PointCloud<PointXYZ>);
	  	vector<Vertices> mesh_vertices;
	  	string mesh_id = "mesh_nurbs";
	  	on_nurbs::Triangulation::convertSurface2PolygonMesh (fit.m_nurbs, mesh, mesh_resolution);

	  	cout << "\nConverting Surface to Vertices"  << endl;

	  	for (unsigned i = 0; i < refinement; i++){
		  fit.refine (0);
		  fit.refine (1);
		  fit.assemble (params);
		  fit.solve ();
		  on_nurbs::Triangulation::convertSurface2Vertices (fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
		  cout << "\nIteration: " << i << endl;
		}

		cout << "\nSurface Fitting"  << endl;
		// surface fitting with final refinement level
		for (unsigned i = 0; i < iterations; i++){
		  fit.assemble (params);
		  fit.solve ();
		  on_nurbs::Triangulation::convertSurface2Vertices (fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
		  cout << "\nIteration: " << i << endl;
		}

		on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
	    curve_params.addCPsAccuracy = 5e-2;
	    curve_params.addCPsIteration = 3;
	    curve_params.maxCPs = 200;
	    curve_params.accuracy = 1e-3;
	    curve_params.iterations = 10;

	    curve_params.param.closest_point_resolution = 0;
	    curve_params.param.closest_point_weight = 1.0;
	    curve_params.param.closest_point_sigma2 = 0.1;
	    curve_params.param.interior_sigma2 = 0.00001;
	    curve_params.param.smooth_concavity = 1.0;
	    curve_params.param.smoothness = 1.0;

	    cout << "\nCurve Fit initializing"  << endl;
	    on_nurbs::NurbsDataCurve2d curve_data;
	    curve_data.interior = data.interior_param;
	    curve_data.interior_weight_function.push_back (true);
	    ON_NurbsCurve curve_nurbs = on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D (order, curve_data.interior);

	    cout << "\nCurve Fitting"  << endl;
	    on_nurbs::FittingCurve2dASDM curve_fit (&curve_data, curve_nurbs);
	    curve_fit.fitting (curve_params);
	    cout << "\nReconstruction done!"<< endl;

	   	on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh (fit.m_nurbs, curve_fit.m_nurbs, mesh, mesh_resolution);

	   	if (fit.m_nurbs.IsValid()){
		    cout << "\nSaving ply file: "  << outputfile << endl;
	    	pcl::io::savePLYFile(outputfile.c_str(), mesh);
	  	}	*/
	}

	return 0;
}