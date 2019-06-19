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
#include "app.h"

int main(int argc, char *argv[])
{
	if (argc != 6)
	{
        printf("Usage ::\n");
		printf("%s [feature_01] [feature_02] [transform_output_txt] [do_initial_matching] [do_cross_check]\n", argv[0]);
		return 0;
	}
	bool initial_matching = (argv[4] == "true") || (argv[4] == "True");
	bool cross_check = (argv[5] == "true") || (argv[5] == "True");
	fgr::CApp app;
	app.ReadFeature(argv[1], false, initial_matching); // source fpfh
	app.ReadFeature(argv[2], true, initial_matching); // target fpfh
	app.NormalizePoints();
	app.AdvancedMatching(cross_check);
	app.OptimizePairwise(cross_check);
	app.WriteTrans(argv[3]);

	return 0;
}
