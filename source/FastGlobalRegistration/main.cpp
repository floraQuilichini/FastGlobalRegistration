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
#include <stdio.h>
#include <chrono>
#include "app.h"

int main(int argc, char *argv[])
{
	/*
	// with fpfh descriptors
	printf("nb arg : %d \n", argc);
	if (argc != 6)
	{
        printf("Usage ::\n");
		printf("%s [feature_01] [feature_02] [transform_output_txt] [do_initial_matching] [do_cross_check]\n", argv[0]);
		return 0;
	}
	bool initial_matching = (std::string(argv[4]) == "true") || (std::string(argv[4]) == "True");
	bool cross_check = (std::string(argv[5]) == "true") || (std::string(argv[5]) == "True");
	fgr::CApp app;
	app.ReadFeature(argv[1], false, initial_matching); // source fpfh
	app.ReadFeature(argv[2], true, initial_matching); // target fpfh
	app.NormalizePoints();
	app.AdvancedMatching(cross_check);
	app.OptimizePairwise(true);
	app.WriteTrans(argv[3]);
	*/

	// with GLS descriptors
	printf("nb arg : %d \n", argc);
	if (argc != 5)
	{
		printf("Usage ::\n");
		printf("%s [source_point_cloud] [target_point_cloud] [triplet_pairs_file] [transform_output_txt]\n", argv[0]);
		return 0;
	}

	fgr::CApp app;
	app.ReadPointCloud(argv[1]); // source ply point cloud
	app.ReadPointCloud(argv[2]); // target ply point cloud
	//app.ReadTriplets(argv[3]);
	app.ReadPairs(argv[3]);
	app.NormalizePoints();
	std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
	//app.TripletConstraint();
	app.PairsConstraint(12);
	std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
	app.OptimizePairwise(true);
	std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
	std::chrono::nanoseconds ns_fgr = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t0);
	std::cout << "It took " << ns_fgr.count() << " nanosecond(s) to run FGR" << std::endl;
	std::chrono::nanoseconds ns_triplet = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0);
	std::cout << "It took " << ns_triplet.count() << " nanosecond(s) to run triplet constraints" << std::endl;
	app.WriteTrans(argv[4]);

	std::cin.get();
	return 0;
}
