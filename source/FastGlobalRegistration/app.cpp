// ----------------------------------------------------------------------------
// -                       Fast Global Registration                           -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) Intel Corporation 2016
// Qianyi Zhou <Qianyi.Zhou@gmail.com>
// Jaesik Park <syncle@gmail.com>
// Vladlen Koltun <vkoltun@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "app.h"

using namespace Eigen;
using namespace std;
using namespace fgr;

std::string CApp::extract_ext(std::string filename)
{
	size_t i = filename.rfind('.', filename.length());
	if (i != std::string::npos) {
		return(filename.substr(i + 1, filename.length() - i));
	}

	return("");
}

float CApp::Median(std::vector<float>::iterator begin, std::vector<float>::iterator end)
{
	std::multiset<float> set;
	for (auto it = begin; it!=end; it++)
		set.insert(*it);
	
	return *std::next(set.begin(), set.size() / 2);
}

float CApp::Mean(std::vector<float>& vec)
{
	float mean = 0.0;
	for (int i = 0; i < (int)vec.size(); i++)
		mean += vec[i];

	return mean/(float)vec.size();
}

std::pair<float, float> CApp::Std(std::vector<float>& vec)
{
	float mean = Mean(vec);
	int nb_samples = vec.size();
	float std = 0.0;
	for (int i = 0; i < nb_samples; i++)
		std += (vec[i] - mean)*(vec[i] - mean);
	std /= (float)nb_samples;

	return std::make_pair(sqrt(std), mean);
}


int CApp::Factorial(int n)
{
	if (n > 1)
		return n*Factorial(n - 1);
	else
		return 1;
}

int CApp::NumberOfCombinations(int n, int p)
{
	if (n < p)
	{
		int temp = n;
		n = p;
		p = temp;
	}

	int factorial_n = Factorial(n);
	int factorial_p = Factorial(p);
	int factorial_n_p = Factorial(n - p);

	return factorial_n / (factorial_p*factorial_n_p);
}

std::tuple<int, int, int> CApp::get_3_different_random_integers(int k)
{
	int rand0 = rand() % k;
	int rand1 = rand() % k;
	int rand2 = rand() % k;
	while (rand1 == rand0)
		rand1 = rand() % k;
	while (rand2 == rand0 || rand2 == rand1)
		rand2 = rand() % k;

	return std::make_tuple(rand0, rand1, rand2);
}

std::tuple<float, float, float, std::vector<int>> CApp::compute_histogram(std::vector<float>& vec, float step)
{
	std::vector<float>::iterator min_it = std::min_element(vec.begin(), vec.end());
	std::vector<float>::iterator max_it = std::max_element(vec.begin(), vec.end());
	float min_abs = std::floor(*min_it / step)*step;
	float max_abs = std::round(*max_it / step)*step;
	std::vector<int> occurences(round((max_abs - min_abs) / step) +1);

	for (int j = 0; j < (int)vec.size(); j++)
	{
		int index = floor(vec[j] / step) - min_abs / step;
		occurences[index] += 1;
	}
	
	return std::make_tuple(min_abs, max_abs, step, occurences);
}

float CApp::compute_most_probable_scale_coeff(std::vector<int>& occurences, int nb_samples, float step, float min_abs_distribution)
{
	float most_probable_value = 0.0;
	for (int i=0; i < (int)occurences.size(); i++)
		most_probable_value +=(float) occurences[i]*(i*step + min_abs_distribution);
	most_probable_value /= (float)nb_samples;
	return most_probable_value;
	
}

float CApp::compute_scale(Eigen::Vector3f& pt1, Eigen::Vector3f& pt2, Eigen::Vector3f& pt3, Eigen::Vector3f& ps1, Eigen::Vector3f& ps2, Eigen::Vector3f& ps3)
{
	float a, b, c, A, B, C;

	a = sqrt((pt1 - pt2).transpose()*(pt1 - pt2));
	b = sqrt((pt2 - pt3).transpose()*(pt2 - pt3));
	c = sqrt((pt1 - pt3).transpose()*(pt1 - pt3));
	A = sqrt((ps1 - ps2).transpose()*(ps1 - ps2));
	B = sqrt((ps3 - ps2).transpose()*(ps3 - ps2));
	C = sqrt((ps1 - ps3).transpose()*(ps1 - ps3));

	float scale = (A / a + B / b + C / c) / 3.0;
	return scale;
}

void CApp::ReadFeature(const char* filepath, bool target, bool initialmatching)
{
	Points pts;
	Feature feat;
	ReadFeature(filepath, pts, feat, target, initialmatching);
	LoadFeature(pts,feat);
}

bool CApp::ReadPointCloud(const char* filepath)
{
	Points pts;
	std::string file_ext = extract_ext(filepath);
	if (file_ext.compare("ply") == 0)
	{
		// read ply file and go to the section where faces are listed
		FILE* fid = fopen(filepath, "r"); // not binary file
		char string_header[50];

		if (fid == NULL)
		{
			std::cerr << "can't open file" << std::endl;
			return false;
		}
		
		// find number of vertices
		int nvertex = 0;

		while (fgets(string_header, 50, fid) != NULL) {
			std::string s(string_header);
			if (s.find("element vertex") != std::string::npos)
				nvertex = std::atoi((s.erase(0, 15)).c_str());
			if (s.find("end_header") != std::string::npos)
				break;
		} ;

		// get points
		for (int v = 0; v < nvertex; v++) {

			Vector3f pts_v;
			float pts_x_v, pts_y_v, pts_z_v, normals_x_v, normals_y_v, normals_z_v;
			fscanf(fid, "%f", &pts_x_v);
			fscanf(fid, "%f", &pts_y_v);
			fscanf(fid, "%f", &pts_z_v);
			fscanf(fid, "%f", &normals_x_v);
			fscanf(fid, "%f", &normals_y_v);
			fscanf(fid, "%f", &normals_z_v);
			pts_v << pts_x_v, pts_y_v, pts_z_v;

			pts.push_back(pts_v);
		}

		// load points 
		pointcloud_.push_back(pts);

		fclose(fid);
		return true;
	}

	else
	{
		// not implemented yet for other types of files
		return false;
	}
}

void CApp::ReadTriplets(const char* filepath)
{
	// read txt file
	FILE* fid = fopen(filepath, "r"); // not binary file

	if (fid == NULL)
		std::cerr << "can't open file" << std::endl;

	// get nb triplet
	int nb_triplets;
	fscanf(fid, "%d", &nb_triplets);
	// get points
	for (int v = 0; v < nb_triplets*3; v++) {

		int target_index, source_index;
		float scale;
		fscanf(fid, "%d", &target_index);
		fscanf(fid, "%d", &source_index);
		fscanf(fid, "%f", &scale);

		pairs_.push_back(std::make_tuple(target_index, source_index, scale));
	}

	fclose(fid);
	
}

int CApp::ReadPairs(const char* filepath)
{
	// read txt file
	FILE* fid = fopen(filepath, "r"); // not binary file

	if (fid == NULL)
		std::cerr << "can't open file" << std::endl;

	// get nb pairs
	int nb_pairs;
	fscanf(fid, "%d", &nb_pairs);
	// get points
	for (int v = 0; v < nb_pairs; v++) {

		int target_index, source_index;
		float scale;
		fscanf(fid, "%d", &target_index);
		fscanf(fid, "%d", &source_index);
		fscanf(fid, "%f", &scale);

		pairs_.push_back(std::make_tuple(target_index, source_index, scale));
	}

	fclose(fid);

	return nb_pairs;
}


void CApp::LoadFeature(const Points& pts, const Feature& feat)
{
	pointcloud_.push_back(pts);
	features_.push_back(feat);
}

void CApp::ReadFeature(const char* filepath, Points& pts, Feature& feat, bool target, bool initialmatching)
{
	printf("ReadFeature ... ");
	FILE* fid = fopen(filepath, "rb");
	int nvertex;
	fread(&nvertex, sizeof(int), 1, fid);
	int ndim;
	fread(&ndim, sizeof(int), 1, fid);

	// read from feature file and fill out pts and feat
	for (int v = 0; v < nvertex; v++)	{

		Vector3f pts_v;
		fread(&pts_v(0), sizeof(float), 3, fid);

		VectorXf feat_v(ndim);
		fread(&feat_v(0), sizeof(float), ndim, fid);

		pts.push_back(pts_v);
		feat.push_back(feat_v);
	}

	// read source and target index matching
	initialmatching_ = initialmatching;

	if (!initialmatching)
	{
		int npair;
		fread(&npair, sizeof(unsigned int), 1, fid);
		std::cout << "nb pairs : " << npair << std::endl;
		//std::cout << "size int : " << sizeof(unsigned int) << std::endl;
		if (target)
		{
			for (int pair_index = 0; pair_index < npair; pair_index++)
			{
				Vector2i ij;
				fread(&ij(0), sizeof(unsigned int), 2, fid);
				corres_ji_.push_back(std::make_pair(ij(0), ij(1)));  //contains correspondence pairs from i to j
				//std::cout << "source index : " << ij(0) << " , target index : " << ij(1) << std::endl;
			}
		}
		else
		{
			for (int pair_index = 0; pair_index < npair; pair_index++)
			{
				Vector2i ij;
				fread(&ij(0), sizeof(unsigned int), 2, fid);
				corres_ij_.push_back(std::make_pair(ij(0), ij(1)));    //contains correspondence pairs from j to i
				std::cout << "source index : " << ij(0) << " , target index : " << ij(1) << std::endl;
			}
		}
	
	}

	fclose(fid);
	printf("%d points with %d feature dimensions.\n", nvertex, ndim);
}

template <typename T>
void CApp::BuildKDTree(const vector<T>& data, KDTree* tree)
{
	int rows, dim;
	rows = (int)data.size();
	dim = (int)data[0].size();
	std::vector<float> dataset(rows * dim);
	flann::Matrix<float> dataset_mat(&dataset[0], rows, dim);
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < dim; j++)
			dataset[i * dim + j] = data[i][j];
	KDTree temp_tree(dataset_mat, flann::KDTreeSingleIndexParams(15));
	temp_tree.buildIndex();
	*tree = temp_tree;
}

template <typename T>
void CApp::SearchKDTree(KDTree* tree, const T& input, 
							std::vector<int>& indices,
							std::vector<float>& dists, int nn)
{
	int rows_t = 1;
	int dim = input.size();

	std::vector<float> query;
	query.resize(rows_t*dim);
	for (int i = 0; i < dim; i++)
		query[i] = input(i);
	flann::Matrix<float> query_mat(&query[0], rows_t, dim);

	indices.resize(rows_t*nn);
	dists.resize(rows_t*nn);
	flann::Matrix<int> indices_mat(&indices[0], rows_t, nn);
	flann::Matrix<float> dists_mat(&dists[0], rows_t, nn);

	tree->knnSearch(query_mat, indices_mat, dists_mat, nn, flann::SearchParams(128));
}

void CApp::AdvancedMatching(bool crosscheck)
{
	int fi = 0;
	int fj = 1;

	printf("Advanced matching : [%d - %d]\n", fi, fj);

	
	bool swapped = false;
	if (initialmatching_)
	{
		if (pointcloud_[fj].size() > pointcloud_[fi].size())
		{
			int temp = fi;
			fi = fj;
			fj = temp;
			swapped = true;
		}
	}

	int nPti = pointcloud_[fi].size();
	int nPtj = pointcloud_[fj].size(); // we have nPtj < nPtj


	///////////////////////////
	/// MATCHING
	///////////////////////////
	bool tuple = true;

	std::vector<int> corres_K, corres_K2;
	std::vector<float> dis;
	std::vector<int> ind;

	std::vector<std::pair<int, int> > corres;
	std::vector<std::pair<int, int> > corres_cross;


	if (initialmatching_)
	{
		///////////////////////////
		/// BUILD FLANNTREE
		///////////////////////////

		KDTree feature_tree_i(flann::KDTreeSingleIndexParams(15));
		BuildKDTree(features_[fi], &feature_tree_i);

		KDTree feature_tree_j(flann::KDTreeSingleIndexParams(15));
		BuildKDTree(features_[fj], &feature_tree_j);

		///////////////////////////
		/// INITIAL MATCHING
		///////////////////////////

		std::vector<int> i_to_j(nPti, -1);
		for (int j = 0; j < nPtj; j++)
		{
			// search among fi-feature array (feature_tree_i) the element that matches with the jth element of fj-feature array (features_[fj][j])
			// matching index in fi-feature array is stored in corres_K
			// dis is the resulting distance (L2 (see l 46 in app.h)) value between jth and ith feature (resp from fj-feature array and fi-feature array)
			SearchKDTree(&feature_tree_i, features_[fj][j], corres_K, dis, 1);
			int i = corres_K[0]; // find index matching in fi-feature array for the jth element of fj-feature array

			// this condition is a sort of reprocity operation : first we find the element in fi-feature array that matches a given element in fj-feature array
			// and then for that same given element in fi-feature array, we find its matching element in fj-feature array
			if (i_to_j[i] == -1) // i_to_j store matching in fj-feature array for the ith element of fi-feature array
								 // the ith element is the one which matched the above j-feature
								 // if the ith element already belongs to a ij pair, then the folowing code is not executed
			{
				SearchKDTree(&feature_tree_j, features_[fi][i], corres_K, dis, 1);
				int ij = corres_K[0];
				i_to_j[i] = ij; // store index matching in fj-feature array for the ith element of fi-feature array
			}
			corres_ji_.push_back(std::pair<int, int>(i, j)); // corres_ji store matching pairs indexes from features_i to features_j
		}

		for (int i = 0; i < nPti; i++)
		{
			if (i_to_j[i] != -1) // take i-features in fi-feature array that have a matching in fj-feature array
				corres_ij_.push_back(std::pair<int, int>(i, i_to_j[i])); // corres_ij store matching pairs indexes from features_j to features_i
		}

		int ncorres_ij = corres_ij_.size();
		int ncorres_ji_ = corres_ji_.size(); // we have corres_ij < or = corres_ji

		// corres = corres_ij + corres_ji;
		for (int i = 0; i < ncorres_ij; ++i)
			corres.push_back(std::pair<int, int>(corres_ij_[i].first, corres_ij_[i].second)); // corres = corres_ij
		for (int j = 0; j < ncorres_ji_; ++j)
			corres.push_back(std::pair<int, int>(corres_ji_[j].first, corres_ji_[j].second)); // corres += corres_ji

		printf("Number of points that remain: %d\n", (int)corres.size());
	}
	
	///////////////////////////
	/// CROSS CHECK
	/// input : corres_ij, corres_ji
	/// output : corres
	/// only keep ij pairs that equal ji pairs
	///////////////////////////
	if (crosscheck)
	{
		printf("\t[cross check] ");

		// build data structure for cross check
		corres.clear(); // clear corres vector
		corres_cross.clear();
		std::vector<std::vector<int> > Mi(nPti);
		std::vector<std::vector<int> > Mj(nPtj);

		int ncorres_ij = corres_ij_.size();
		int ncorres_ji = corres_ji_.size();
		int ci, cj;
		for (int i = 0; i < ncorres_ij; ++i)
		{
			ci = corres_ij_[i].first;
			cj = corres_ij_[i].second;
			Mi[ci].push_back(cj); // store jth-index at Mi ith-index --> vector sorted along i indexes 
		}
		for (int j = 0; j < ncorres_ji; ++j)
		{
			ci = corres_ji_[j].first;
			cj = corres_ji_[j].second;
			Mj[cj].push_back(ci); // store ith-index at Mj jth-index --> vector sorted along j indexes
		}

		// cross check
		for (int i = 0; i < nPti; ++i)
		{
			for (int ii = 0; ii < (int)Mi[i].size(); ++ii)
			{
				int j = Mi[i][ii];
				for (int jj = 0; jj < (int)Mj[j].size(); ++jj)
				{
					if (Mj[j][jj] == i) // if cross-checked
					{
						corres.push_back(std::pair<int, int>(i, j));
						printf("i-j pairs: %d %d\n", i, j);
						corres_cross.push_back(std::pair<int, int>(i, j)); // store ij pair in corres_cross
					}
				}
			}
		}
		printf("Number of points that remain after cross-check: %d\n", (int)corres.size());
	}
	else
	{
		corres = corres_ij_;
		printf("Number of points that remain after cross-check: %d\n", (int)corres.size());
	}

	///////////////////////////
	/// TUPLE CONSTRAINT
	/// input : corres
	/// output : corres
	///////////////////////////
	if (tuple)
	{
		srand(time(NULL));

		printf("\t[tuple constraint] ");
		int rand0, rand1, rand2;
		int idi0, idi1, idi2;
		int idj0, idj1, idj2;
		float scale = tuple_scale_;
		int ncorr = corres.size();
		int number_of_trial = ncorr * 100;
		std::vector<std::pair<int, int> > corres_tuple;

		int cnt = 0;
		int i;
		for (i = 0; i < number_of_trial; i++)
		{
			rand0 = rand() % ncorr;
			rand1 = rand() % ncorr;
			rand2 = rand() % ncorr;

			idi0 = corres[rand0].first;
			idj0 = corres[rand0].second;
			idi1 = corres[rand1].first;
			idj1 = corres[rand1].second;
			idi2 = corres[rand2].first;
			idj2 = corres[rand2].second;

			// collect 3 points from i-th fragment
			Eigen::Vector3f pti0 = pointcloud_[fi][idi0];
			Eigen::Vector3f pti1 = pointcloud_[fi][idi1];
			Eigen::Vector3f pti2 = pointcloud_[fi][idi2];

			float li0 = (pti0 - pti1).norm(); // compute point to point distance
			float li1 = (pti1 - pti2).norm();
			float li2 = (pti2 - pti0).norm();

			// collect 3 points from j-th fragment
			Eigen::Vector3f ptj0 = pointcloud_[fj][idj0];
			Eigen::Vector3f ptj1 = pointcloud_[fj][idj1];
			Eigen::Vector3f ptj2 = pointcloud_[fj][idj2];

			float lj0 = (ptj0 - ptj1).norm(); // compute point to point distance
			float lj1 = (ptj1 - ptj2).norm();
			float lj2 = (ptj2 - ptj0).norm();

			if ((li0 * scale < lj0) && (lj0 < li0 / scale) &&
				(li1 * scale < lj1) && (lj1 < li1 / scale) &&
				(li2 * scale < lj2) && (lj2 < li2 / scale)) // if distance between i-points are roughly the same as distance between j points
			{
				corres_tuple.push_back(std::pair<int, int>(idi0, idj0)); // keep these three points to compute T
				corres_tuple.push_back(std::pair<int, int>(idi1, idj1));
				corres_tuple.push_back(std::pair<int, int>(idi2, idj2));
				cnt++;
			}

			if (cnt >= tuple_max_cnt_)  // we have at max tuple_max_cnt_*3 pairs of points to estimate T
				break;
		}

		printf("%d tuples (%d trial, %d actual).\n", cnt, number_of_trial, i);
		corres.clear();

		for (int i = 0; i < (int)corres_tuple.size(); ++i)
			corres.push_back(std::pair<int, int>(corres_tuple[i].first, corres_tuple[i].second));
	}

	if (swapped)
	{
		std::vector<std::pair<int, int> > temp;
		for (int i = 0; i < (int)corres.size(); i++)
			temp.push_back(std::pair<int, int>(corres[i].second, corres[i].first));
		corres.clear();
		corres = temp;
	}

	printf("\t[final] matches %d.\n", (int)corres.size());
	corres_ = corres;
}

void CApp::TripletConstraint()
{
	float scale_ratio = tuple_scale_;
	std::vector<float> scale_coeff_vec;
	std::vector<std::tuple<int, int, float> > corres_tuple;
	int i0, i1, i2, j0, j1, j2;
	int cnt = 0;
	int nb_triplets = (int)pairs_.size() / 3;
	for (int i = 0; i < nb_triplets; i++)
	{
		// get scale source/target
		float scale_coeff  = std::get<2>(pairs_[3 * i]);

		// get source and target indices
		j0 = std::get<0>(pairs_[3*i]);
		i0 = std::get<1>(pairs_[3*i]);
		j1 = std::get<0>(pairs_[3*i+1]);
		i1 = std::get<1>(pairs_[3*i+1]);
		j2 = std::get<0>(pairs_[3*i+2]);
		i2 = std::get<1>(pairs_[3*i+2]);

		// collect triplet of points from source fragment
		Eigen::Vector3f pti0 = pointcloud_[0][i0];
		Eigen::Vector3f pti1 = pointcloud_[0][i1];
		Eigen::Vector3f pti2 = pointcloud_[0][i2];

		float li0 = (pti0 - pti1).norm(); // compute point to point distance
		float li1 = (pti1 - pti2).norm();
		float li2 = (pti2 - pti0).norm();

		// collect triplet of points from target fragment
		Eigen::Vector3f ptj0 = pointcloud_[1][j0];
		Eigen::Vector3f ptj1 = pointcloud_[1][j1];
		Eigen::Vector3f ptj2 = pointcloud_[1][j2];

		float lj0 = scale_coeff*(ptj0 - ptj1).norm(); // compute point to point distance
		float lj1 = scale_coeff*(ptj1 - ptj2).norm();
		float lj2 = scale_coeff*(ptj2 - ptj0).norm();

		if ((li0 * scale_ratio < lj0) && (lj0 < li0 / scale_ratio) &&
			(li1 * scale_ratio < lj1) && (lj1 < li1 / scale_ratio) &&
			(li2 * scale_ratio < lj2) && (lj2 < li2 / scale_ratio)) // if distance between i-points are roughly the same as distance between j points
		{
			corres_tuple.push_back(std::tuple<int, int, float>(i0, j0, scale_coeff)); // keep these three points to compute T
			corres_tuple.push_back(std::tuple<int, int, float>(i1, j1, scale_coeff));
			corres_tuple.push_back(std::tuple<int, int, float>(i2, j2, scale_coeff));

			scale_coeff_vec.push_back(scale_coeff);
			cnt++;
		}

		if (cnt >= tuple_max_cnt_)  // we have at max tuple_max_cnt_*3 pairs of points to estimate T
			break;
	}

	float step = 0.1;
	std::tuple<float, float, float, std::vector<int>> hist = compute_histogram(scale_coeff_vec, step);
	std::vector<int>& occurences = std::get<3>(hist);
	float min_abs = std::get<0>(hist);
	optimal_scale_coeff_ = std::distance(occurences.begin(), std::max_element(occurences.begin(), occurences.end()))*step + min_abs;
	//optimal_scale_coeff_ = Median(scale_coeff_vec.begin(), scale_coeff_vec.end());
	//optimal_scale_coeff_ = Mean(scale_coeff_vec);

	/*optimal_scale_coeff_ = 2.0;
	corres_.clear();
	corres_.push_back(std::make_pair(50, 50));
	corres_.push_back(std::make_pair(40, 40));
	corres_.push_back(std::make_pair(30, 30));
	corres_.push_back(std::make_pair(20, 20));
	corres_.push_back(std::make_pair(100, 100));
	corres_.push_back(std::make_pair(21, 21));
	corres_.push_back(std::make_pair(11, 11));
	corres_.push_back(std::make_pair(67, 67));
	corres_.push_back(std::make_pair(25, 25));
	corres_.push_back(std::make_pair(36, 36));
	corres_.push_back(std::make_pair(48, 48));
	corres_.push_back(std::make_pair(75, 75));*/

	corres_.clear();
	for (int i = 0; i < (int)corres_tuple.size(); ++i)
	{
		float scale_coeff = std::get<2>(corres_tuple[i]);
		if ( abs(scale_coeff - optimal_scale_coeff_) <= 0.2)
			corres_.push_back(std::pair<int, int>(std::get<0>(corres_tuple[i]), std::get<1>(corres_tuple[i])));
	}

}


void CApp::PairsConstraint(int k)
{
	float scale_ratio = tuple_scale_;
	std::vector<float> scale_coeff_vec;
	std::vector<std::tuple<int, int, float> > corres_tuple;
	int i0, i1, i2, j0, j1, j2;
	int cnt = 0;
	int nb_ktuples = (int)pairs_.size() / k;
	tuple_max_cnt_ = NumberOfCombinations(k, 3) * nb_ktuples;
	int number_of_trial = 1000;

	for (int i = 0; i < number_of_trial; i++)
	{
		// random selection of the ktuple
		int kt = rand() % nb_ktuples;

		// get source and target indices
		std::tuple<int, int, int> three_integers = get_3_different_random_integers(k);

		j0 = std::get<0>(pairs_[kt*k + std::get<0>(three_integers)]);
		i0 = std::get<1>(pairs_[kt*k + std::get<0>(three_integers)]);
		j1 = std::get<0>(pairs_[kt*k + std::get<1>(three_integers)]);
		i1 = std::get<1>(pairs_[kt*k + std::get<1>(three_integers)]);
		j2 = std::get<0>(pairs_[kt*k + std::get<2>(three_integers)]);
		i2 = std::get<1>(pairs_[kt*k + std::get<2>(three_integers)]);

		// collect triplet of points from source fragment
		Eigen::Vector3f pti0 = pointcloud_[0][i0];
		Eigen::Vector3f pti1 = pointcloud_[0][i1];
		Eigen::Vector3f pti2 = pointcloud_[0][i2];

		float li0 = (pti0 - pti1).norm(); // compute point to point distance
		float li1 = (pti1 - pti2).norm();
		float li2 = (pti2 - pti0).norm();

		// collect triplet of points from target fragment
		Eigen::Vector3f ptj0 = pointcloud_[1][j0];
		Eigen::Vector3f ptj1 = pointcloud_[1][j1];
		Eigen::Vector3f ptj2 = pointcloud_[1][j2];

		// get scale source/target
		//float scale_coeff = (std::get<2>(pairs_[kt*k + std::get<0>(three_integers)]) + std::get<2>(pairs_[kt*k + std::get<1>(three_integers)]) + std::get<2>(pairs_[kt*k + std::get<2>(three_integers)]))/3.0;
		float scale_coeff = compute_scale(ptj0, ptj1, ptj2, pti0, pti1, pti2);

		// tuple test
		/*float lj0 = scale_coeff*(ptj0 - ptj1).norm(); // compute point to point distance
		float lj1 = scale_coeff*(ptj1 - ptj2).norm();
		float lj2 = scale_coeff*(ptj2 - ptj0).norm();

		if ((li0 * scale_ratio < lj0) && (lj0 < li0 / scale_ratio) &&
			(li1 * scale_ratio < lj1) && (lj1 < li1 / scale_ratio) &&
			(li2 * scale_ratio < lj2) && (lj2 < li2 / scale_ratio)) // if distance between i-points are roughly the same as distance between j points
		{*/
			corres_tuple.push_back(std::tuple<int, int, float>(i0, j0, std::get<2>(pairs_[kt*k + std::get<0>(three_integers)]))); // keep these three points to compute T
			corres_tuple.push_back(std::tuple<int, int, float>(i1, j1, std::get<2>(pairs_[kt*k + std::get<1>(three_integers)])));
			corres_tuple.push_back(std::tuple<int, int, float>(i2, j2, std::get<2>(pairs_[kt*k + std::get<2>(three_integers)])));

			/*scale_coeff_vec.push_back(std::get<2>(pairs_[kt*k + std::get<0>(three_integers)]));
			scale_coeff_vec.push_back(std::get<2>(pairs_[kt*k + std::get<1>(three_integers)]));
			scale_coeff_vec.push_back(std::get<2>(pairs_[kt*k + std::get<2>(three_integers)]));
			scale_coeff_vec.push_back(scale_coeff);*/

			cnt++;
		//}

		if (cnt >= tuple_max_cnt_)  // we have at max tuple_max_cnt_*3 pairs of points to estimate T
			break;
	}

	/*float step = 0.1;
	std::pair<float, float> std_mean = Std(scale_coeff_vec);
	//optimal_scale_coeff_ = std_mean.second;
	std::tuple<float, float, float, std::vector<int>> hist = compute_histogram(scale_coeff_vec, step);
	std::vector<int>& occurences = std::get<3>(hist);
	float min_abs = std::get<0>(hist);
	optimal_scale_coeff_ = compute_most_probable_scale_coeff(occurences, scale_coeff_vec.size(), step, min_abs);*/
	optimal_scale_coeff_ = 1.0;
	//optimal_scale_coeff_ = std::distance(occurences.begin(), std::max_element(occurences.begin(), occurences.end()))*step + min_abs;
	//optimal_scale_coeff_ = Median(scale_coeff_vec.begin(), scale_coeff_vec.end());
	//optimal_scale_coeff_ = Mean(scale_coeff_vec);

	corres_.clear();
	for (int i = 0; i < (int)corres_tuple.size(); ++i)
	{
		/*float scale_coeff = std::get<2>(corres_tuple[i]);
		if (abs(scale_coeff - optimal_scale_coeff_) <= std_mean.first)*/
			corres_.push_back(std::pair<int, int>(std::get<0>(corres_tuple[i]), std::get<1>(corres_tuple[i])));
	}

}



// Normalize scale of points.
// X' = (X-\mu)/scale
void CApp::NormalizePoints()
{
	int num = 2;
	float scale = 0;

	Means.clear();

	for (int i = 0; i < num; ++i)
	{
		float max_scale = 0;

		// compute mean
		Vector3f mean;
		mean.setZero();

		int npti = pointcloud_[i].size();
		for (int ii = 0; ii < npti; ++ii)
		{
			Eigen::Vector3f p(pointcloud_[i][ii](0), pointcloud_[i][ii](1), pointcloud_[i][ii](2));
			mean = mean + p;
		}
		mean = mean / npti;
		Means.push_back(mean); // compute the mean point of point cloud for each of the two point clouds

		printf("normalize points :: mean[%d] = [%f %f %f]\n", i, mean(0), mean(1), mean(2));

		for (int ii = 0; ii < npti; ++ii) // center each of the two point clouds
		{
			pointcloud_[i][ii](0) -= mean(0);
			pointcloud_[i][ii](1) -= mean(1);
			pointcloud_[i][ii](2) -= mean(2);
		}

		// compute scale
		for (int ii = 0; ii < npti; ++ii)
		{
			Eigen::Vector3f p(pointcloud_[i][ii](0), pointcloud_[i][ii](1), pointcloud_[i][ii](2));
			float temp = p.norm(); // because we extract mean in the previous stage.
			if (temp > max_scale)
				max_scale = temp;
		}

		if (max_scale > scale) // compute max point distance from origin (scale) between the two point clouds
			scale = max_scale;
	}

	//// mean of the scale variation
	if (use_absolute_scale_) {
		GlobalScale = 1.0f;
		StartScale = scale;
	} else {
		GlobalScale = scale; // second choice: we keep the maximum scale.
		StartScale = 1.0f;
	}
	printf("normalize points :: global scale : %f\n", GlobalScale);

	for (int i = 0; i < num; ++i) // points of both of the point clouds are normalized
	{
		int npti = pointcloud_[i].size();
		for (int ii = 0; ii < npti; ++ii)
		{
			pointcloud_[i][ii](0) /= GlobalScale;
			pointcloud_[i][ii](1) /= GlobalScale;
			pointcloud_[i][ii](2) /= GlobalScale;
		}
	}
}

double CApp::OptimizePairwise(bool decrease_mu_)
{
	printf("Pairwise rigid pose optimization\n");

	bool flag_update_rt_matrix_with_scale = false;
	bool flag_update_rt_matrix = false;
	bool flag_update_scale_matrix = false;
	bool flag_update_ds = true;
	bool flag_update_diwt = true;

	double par;
	int numIter = iteration_number_;
	TransOutput_ = Eigen::Matrix4f::Identity();

	par = StartScale;

	int i = 0;
	int j = 1;

	// rescale point cloud
	int npcj = pointcloud_[j].size();
	for (int cnt = 0; cnt < npcj; cnt++)
		pointcloud_[j][cnt] *= optimal_scale_coeff_;

	// make another copy of pointcloud_[j].
	Points pcj_copy;
	pcj_copy.resize(npcj);
	for (int cnt = 0; cnt < npcj; cnt++)
		pcj_copy[cnt] = pointcloud_[j][cnt];

	if (corres_.size() < 10)
		return -1;

	std::vector<double> s(corres_.size(), 1.0);

	Eigen::Matrix4f trans;
	trans.setIdentity();

	for (int itr = 0; itr < numIter; itr++) {

		if (itr % 20 == 19)
		{
			flag_update_rt_matrix_with_scale = true;
			flag_update_rt_matrix = false;
			flag_update_scale_matrix = false;
			flag_update_diwt = true;
			flag_update_ds = true;
		}
		else
		{
			if (itr % 5 != 4)
			{
				flag_update_rt_matrix_with_scale = false;
				flag_update_rt_matrix = true;
				flag_update_scale_matrix = false;
				flag_update_diwt = true;
				flag_update_ds = false;
			}
			else
			{
				flag_update_rt_matrix_with_scale = false;
				flag_update_rt_matrix = false;
				flag_update_scale_matrix = true;
				flag_update_diwt = false;
				flag_update_ds = true;
			}
		}

		// graduated non-convexity.
		if (decrease_mu_)
		{
			if (itr % 4 == 0 && par > max_corr_dist_) {
				par /= div_factor_;
			}
		}

		const int nvariable = 6;	// 3 for rotation and 3 for translation
		Eigen::MatrixXd JTJ(nvariable, nvariable);
		Eigen::MatrixXd JTr(nvariable, 1);
		Eigen::MatrixXd J(nvariable, 1);
		Eigen::MatrixXd JscaleTJscale(3, 3);
		Eigen::MatrixXd JscaleTr(3, 1);
		Eigen::MatrixXd Jscale(3, 1);
		JscaleTJscale.setZero();
		JscaleTr.setZero();
		JTJ.setZero();
		JTr.setZero();
		Eigen::Matrix3d scale_mat = Eigen::Matrix3d::Identity();

		double r;
		double r2 = 0.0;

		for (int c = 0; c < (int)corres_.size(); c++) {
			int ii = corres_[c].first;
			int jj = corres_[c].second;
			Eigen::Vector3f p, q;
			p = pointcloud_[i][ii];
			int c2 = c;

			// estimation de la rotation et translation
			if (flag_update_diwt)
			{
				q = pcj_copy[jj];
				Eigen::Vector3f rpq = p - q;  // rpq : residual between p and q transformed

				// estimation du delta line process
				float temp = par / (rpq.dot(rpq) + par);
				s[c2] = temp * temp; // line process lpq


				// estimation du delta rot et trans
				J.setZero();
				//J(0) = -q(0);
				J(1) = -q(2);
				J(2) = q(1);
				J(3) = -1;
				r = rpq(0);
				JTJ += J * J.transpose() * s[c2];
				JTr += J * r * s[c2];
				r2 += r * r * s[c2];

				J.setZero();
				//J(0) = -q(1);
				J(2) = -q(0);
				J(0) = q(2);
				J(4) = -1;
				r = rpq(1);
				JTJ += J * J.transpose() * s[c2];
				JTr += J * r * s[c2];
				r2 += r * r * s[c2];

				J.setZero();
				//J(0) = -q(2);
				J(0) = -q(1);
				J(1) = q(0);
				J(5) = -1;
				r = rpq(2);
				JTJ += J * J.transpose() * s[c2];
				JTr += J * r * s[c2];
				r2 += r * r * s[c2];

				r2 += (par * (1.0 - sqrt(s[c2])) * (1.0 - sqrt(s[c2])));
			}

			// estimation du scale
			if (flag_update_ds)
			{
				q = pcj_copy[jj];
				Eigen::Vector3f rpq = p - q;  // rpq : residual between p and q transformed

				// estimation du delta line process
				float temp = par / (rpq.dot(rpq) + par);
				s[c2] = temp * temp; // line process lpq

				// estimation du delta scale
				Jscale.setZero();
				Jscale(0) = -q(0);
				r = rpq(0);
				JscaleTJscale += Jscale * Jscale.transpose() * s[c2];
				JscaleTr += Jscale * r * s[c2];

				Jscale.setZero();
				Jscale(1) = -q(1);
				r = rpq(1);
				JscaleTJscale += Jscale * Jscale.transpose() * s[c2];
				JscaleTr += Jscale * r * s[c2];

				Jscale.setZero();
				Jscale(2) = -q(2);
				r = rpq(2);
				JscaleTJscale += Jscale * Jscale.transpose() * s[c2];
				JscaleTr += Jscale * r * s[c2];
			}
		}


		if (flag_update_rt_matrix_with_scale)
		{

			Eigen::MatrixXd result(nvariable, 1);
			result = -JTJ.llt().solve(JTr);
			Eigen::MatrixXd result_scale(3, 1);
			result_scale = -JscaleTJscale.llt().solve(JscaleTr);

			std::cout << "result_scale : " << result_scale.transpose() << std::endl;
			std::cout << "matrix scale intermediate : " << scale_mat*Eigen::Scaling(1.0 + result_scale(0), 1.0 + result_scale(1), 1.0 + result_scale(2)).toDenseMatrix() << std::endl;


			scale_mat *= Eigen::Scaling(1.0 + result_scale(0), 1.0 + result_scale(1), 1.0 + result_scale(2)).toDenseMatrix();

			Eigen::Affine3d aff_mat(Eigen::Affine3d::Identity());
			aff_mat.linear() = (Eigen::Matrix3d) Eigen::AngleAxisd(result(2), Eigen::Vector3d::UnitZ())
				* Eigen::AngleAxisd(result(1), Eigen::Vector3d::UnitY())
				* Eigen::AngleAxisd(result(0), Eigen::Vector3d::UnitX())
				//*Eigen::Scaling(1.0, 1.0, 1.0);
				*scale_mat;
			aff_mat.translation() = Eigen::Vector3d(result(3), result(4), result(5));


			Eigen::Matrix4f delta = aff_mat.matrix().cast<float>();

			trans = delta * trans;
			TransformPoints(pcj_copy, delta);
		}
		else
		{
			if (flag_update_rt_matrix)
			{
				Eigen::MatrixXd result(nvariable, 1);
				result = -JTJ.llt().solve(JTr);

				Eigen::Affine3d aff_mat(Eigen::Affine3d::Identity());
				aff_mat.linear() = (Eigen::Matrix3d) Eigen::AngleAxisd(result(2), Eigen::Vector3d::UnitZ())
					* Eigen::AngleAxisd(result(1), Eigen::Vector3d::UnitY())
					* Eigen::AngleAxisd(result(0), Eigen::Vector3d::UnitX());
				aff_mat.translation() = Eigen::Vector3d(result(3), result(4), result(5));

				Eigen::Matrix4f delta = aff_mat.matrix().cast<float>();

				trans = delta * trans;
				TransformPoints(pcj_copy, delta);

			}
			else
			{
				Eigen::MatrixXd result_scale(3, 1);
				result_scale = -JscaleTJscale.llt().solve(JscaleTr);

				std::cout << "result_scale : " << result_scale.transpose() << std::endl;
				std::cout << "matrix scale intermediate : " << scale_mat*Eigen::Scaling(1.0 + result_scale(0), 1.0 + result_scale(1), 1.0 + result_scale(2)).toDenseMatrix() << std::endl;

				Eigen::Affine3d aff_mat(Eigen::Affine3d::Identity());
				scale_mat *= Eigen::Scaling(1.0 + result_scale(0), 1.0 + result_scale(1), 1.0 + result_scale(2)).toDenseMatrix();
				aff_mat.linear() = scale_mat;
				Eigen::Matrix4f delta = aff_mat.matrix().cast<float>();

				trans = delta * trans;
				TransformPoints(pcj_copy, delta);
			}

		}


	}

	TransOutput_ = trans * TransOutput_;
	return par;
}

void CApp::TransformPoints(Points& points, const Eigen::Matrix4f& Trans)
{
	int npc = (int)points.size();
	Matrix3f R = Trans.block<3, 3>(0, 0);
	Vector3f t = Trans.block<3, 1>(0, 3);
	Vector3f temp;
	for (int cnt = 0; cnt < npc; cnt++) {
		temp = R * points[cnt] + t;
		points[cnt] = temp;
	}
}

Eigen::Matrix4f CApp::GetOutputTrans()
{
	Eigen::Matrix3f R;
	Eigen::Vector3f t;
	R = TransOutput_.block<3, 3>(0, 0)*(optimal_scale_coeff_*Eigen::Matrix3f::Identity());
	t = TransOutput_.block<3, 1>(0, 3);

	Eigen::Matrix4f transtemp;
	transtemp.fill(0.0f);

	transtemp.block<3, 3>(0, 0) = R;
	transtemp.block<3, 1>(0, 3) = -R*Means[1] + t*GlobalScale + Means[0];
	transtemp(3, 3) = 1;
	
	return transtemp;
}
	
void CApp::WriteTrans(const char* filepath)
{
	FILE* fid = fopen(filepath, "w");

	// Below line indicates how the transformation matrix aligns two point clouds
	// e.g. T * pointcloud_[1] is aligned with pointcloud_[0].
	// '2' indicates that there are two point cloud fragments.
	fprintf(fid, "0 1 2\n");

	Eigen::Matrix4f transtemp = GetOutputTrans();

	fprintf(fid, "%.10f %.10f %.10f %.10f\n", transtemp(0, 0), transtemp(0, 1), transtemp(0, 2), transtemp(0, 3));
	fprintf(fid, "%.10f %.10f %.10f %.10f\n", transtemp(1, 0), transtemp(1, 1), transtemp(1, 2), transtemp(1, 3));
	fprintf(fid, "%.10f %.10f %.10f %.10f\n", transtemp(2, 0), transtemp(2, 1), transtemp(2, 2), transtemp(2, 3));
	fprintf(fid, "%.10f %.10f %.10f %.10f\n", 0.0f, 0.0f, 0.0f, 1.0f);

	fclose(fid);
}

Eigen::Matrix4f CApp::ReadTrans(const char* filename)
{
	Eigen::Matrix4f temp;
	temp.fill(0);
	int temp0, temp1, temp2, cnt = 0;
	FILE* fid = fopen(filename, "r");
	while (fscanf(fid, "%d %d %d", &temp0, &temp1, &temp2) == 3)
	{
		for (int j = 0; j < 4; j++)
		{
			float a, b, c, d;
			fscanf(fid, "%f %f %f %f", &a, &b, &c, &d);
			temp(j, 0) = a;
			temp(j, 1) = b;
			temp(j, 2) = c;
			temp(j, 3) = d;
		}
	}
	return temp;
}

void CApp::BuildDenseCorrespondence(const Eigen::Matrix4f& trans, 
		Correspondences& corres)
{   
	int fi = 0;
	int fj = 1;
	Points pci = pointcloud_[fi];
	Points pcj = pointcloud_[fj];
	TransformPoints(pcj, trans);

	KDTree feature_tree_i(flann::KDTreeSingleIndexParams(15));
	BuildKDTree(pci, &feature_tree_i);
	std::vector<int> ind;
	std::vector<float> dist;
	corres.clear();
	for (int j = 0; j < (int)pcj.size(); ++j)
	{
		SearchKDTree(&feature_tree_i, pcj[j], ind, dist, 1);
		float dist_j = sqrt(dist[0]);
		if (dist_j / GlobalScale < max_corr_dist_ / 2.0)
			corres.push_back(std::pair<int, int>(ind[0], j));
	}
}

void CApp::Evaluation(const char* gth, const char* estimation, const char *output)
{
	float inlier_ratio = -1.0f;
	float overlapping_ratio = -1.0f;

	int fi = 0;
	int fj = 1;

	std::vector<std::pair<int, int> > corres;
	Eigen::Matrix4f gth_trans = ReadTrans(gth);
	BuildDenseCorrespondence(gth_trans, corres);
	printf("Groundtruth correspondences [%d-%d] : %d\n", fi, fj, 
			(int)corres.size());

	int ncorres = corres.size();
	float err_mean = 0.0f;

	Points pci = pointcloud_[fi];
	Points pcj = pointcloud_[fj];
	Eigen::Matrix4f est_trans = ReadTrans(estimation);
	std::vector<float> error;
	for (int i = 0; i < ncorres; ++i)
	{
		int idi = corres[i].first;
		int idj = corres[i].second;
		Eigen::Vector4f pi(pci[idi](0), pci[idi](1), pci[idi](2), 1);
		Eigen::Vector4f pj(pcj[idj](0), pcj[idj](1), pcj[idj](2), 1);
		Eigen::Vector4f pjt = est_trans*pj;
		float errtemp = (pi - pjt).norm();
		error.push_back(errtemp);
		// this is based on the RMSE defined in
		// https://en.wikipedia.org/wiki/Root-mean-square_deviation
		errtemp = errtemp * errtemp;
		err_mean += errtemp;
	}
	err_mean /= ncorres; // this is MSE = mean(d^2)
	err_mean = sqrt(err_mean); // this is RMSE = sqrt(MSE)
	printf("mean error : %0.4e\n", err_mean);

	//overlapping_ratio = (float)ncorres / min(
	//		pointcloud_[fj].size(), pointcloud_[fj].size());
	overlapping_ratio = (float)ncorres / pointcloud_[fj].size();
	
	// write errors
	FILE* fid = fopen(output, "w");
	fprintf(fid, "%d %d %e %e %e\n", fi, fj, err_mean, 
			inlier_ratio, overlapping_ratio);
	fclose(fid);
}
