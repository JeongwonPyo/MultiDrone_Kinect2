#include <iostream>
#include <fstream>

// OpenCV
#include "opencv2\opencv.hpp"
#include "opencv2\core.hpp"
#include "opencv2\imgproc.hpp"
#include "opencv2\videoio.hpp"
#include "opencv2\highgui.hpp"
#include "opencv2\imgproc\types_c.h"
#include "opencv2\imgproc\imgproc_c.h"

#include <strsafe.h>
#include <stdarg.h>
#include <math.h>
#include <cmath>
#include <limits>
#include <Wincodec.h>
#include "kinect.h"

#pragma comment(lib,"Windowscodecs.lib")

using namespace std;

using namespace cv;


// --------------------------------------------------------------------------
// 
//	From Kinect Source
// 
// --------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//											Added																				//////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct square_Data{

	Point left_Up;
	Point right_Down;
	vector<Point> Data;

};

//////////////////////////////	Tinning	///////////////////////////////////////////////////////////////////////////
// From this site : http://opencv-code.com/quick-tips/implementation-of-thinning-algorithm-in-opencv/
void thinningIteration(cv::Mat& im, int iter)
{
	cv::Mat marker = cv::Mat::zeros(im.size(), CV_8UC1);

	for (int i = 1; i < im.rows - 1; i++)
	{
		for (int j = 1; j < im.cols - 1; j++)
		{
			uchar p2 = im.at<uchar>(i - 1, j);
			uchar p3 = im.at<uchar>(i - 1, j + 1);
			uchar p4 = im.at<uchar>(i, j + 1);
			uchar p5 = im.at<uchar>(i + 1, j + 1);
			uchar p6 = im.at<uchar>(i + 1, j);
			uchar p7 = im.at<uchar>(i + 1, j - 1);
			uchar p8 = im.at<uchar>(i, j - 1);
			uchar p9 = im.at<uchar>(i - 1, j - 1);

			int A = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
				(p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
				(p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
				(p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
			int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
			int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
			int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

			if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
				marker.at<uchar>(i, j) = 1;
		}
	}

	im &= ~marker;
}

/**
* Function for thinning the given binary image
*
* @param  im  Binary image with range = 0-255
*/
void thinning(cv::Mat& im)
{
	im /= 255;

	cv::Mat prev = cv::Mat::zeros(im.size(), CV_8UC1);
	cv::Mat diff;

	do {
		thinningIteration(im, 0);
		thinningIteration(im, 1);
		cv::absdiff(im, prev, diff);
		im.copyTo(prev);
	} while (cv::countNonZero(diff) > 0);

	im *= 255;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////	Linear Regression	////////////////////////////////////////////////////////////////
// All this i made. If any problem, contact me : jungwon900224@gmail.com
Mat Multiple_Matrix(Mat front, Mat behind)
{
	Mat output_Result = Mat::zeros(front.size().height, behind.size().width, CV_64F);

	if (front.size().width != behind.size().height)
	{
		cout << "Fail to multiple matrix" << endl;

		return output_Result;
	}
	else
	{
		for (int i = 0; i < front.size().height; i++)
		{
			for (int j = 0; j < behind.size().width; j++)
			{
				for (int k = 0; k < front.size().width; k++)
				{
					output_Result.at<double>(i, j) += front.at<double>(i, k)*behind.at<double>(k, j);
				}

			}
		}

		return output_Result;
	}

	return output_Result;
}
double Determinant_Matrix(Mat origin)
{
	Mat temp_matrix = origin.clone();

	Mat stack_matrix = origin.clone();

	int trans_temp_int = 0;

	int trans_count = 0;

	double trans_temp = 0.0;

	double output_Result = 1.0;


	if (origin.size().height != origin.size().width)
	{
		cout << "Fail to convert inverse matrix" << endl;

		//return output_Result;
	}
	else
	{
		for (int i = 0; i < temp_matrix.size().height - 1; i++)
		{
			if ((temp_matrix.at<double>(i, i) == 0) && (i != 0))
			{
				for (int l = i + 1; l < temp_matrix.size().height; l++)
				{
					if (temp_matrix.at<double>(l, i) != 0)
					{
						trans_temp_int = l;

						break;
					}
				}
				//cout << "trans_temp_int : " << trans_temp_int <<endl;

				for (int m = 0; m < temp_matrix.size().width; m++)
				{
					//cout << temp_matrix.at<double>(i, m) << " " << temp_matrix.at<double>(trans_temp_int, m) <<endl;

					trans_temp = temp_matrix.at<double>(i, m);
					temp_matrix.at<double>(i, m) = temp_matrix.at<double>(trans_temp_int, m);
					temp_matrix.at<double>(trans_temp_int, m) = trans_temp;

					//cout << temp_matrix.at<double>(i, m) << " " << temp_matrix.at<double>(trans_temp_int, m) <<endl;
				}

				stack_matrix = temp_matrix.clone();

				trans_count++;
			}
			for (int j = i; j < temp_matrix.size().width; j++)
			{
				for (int k = 1; (i + k) < temp_matrix.size().height; k++)
				{
					stack_matrix.at<double>(i + k, j) = temp_matrix.at<double>(i + k, j) - (temp_matrix.at<double>(i, j) * (temp_matrix.at<double>(i + k, i) / temp_matrix.at<double>(i, i)));
				}
			}

			temp_matrix = stack_matrix.clone();

		}

		for (int m = 0; m < temp_matrix.size().width; m++)
		{
			output_Result *= temp_matrix.at<double>(m, m);
		}

		output_Result = pow(-1.0, double(2 + trans_count)) * output_Result;

	}

	return output_Result;

	//return temp_matrix;

}

Mat Transpose_Matrix(Mat origin)
{
	Mat output_Result = Mat::zeros(origin.size().width, origin.size().height, CV_64F);

	for (int i = 0; i < output_Result.size().height; i++)
	{
		for (int j = 0; j < output_Result.size().width; j++)
		{
			output_Result.at<double>(i, j) = origin.at<double>(j, i);
		}
	}

	return output_Result;
}

Mat Cofector_Matrix(Mat origin)
{
	Mat output_Result = Mat::zeros(origin.size().height, origin.size().width, CV_64F);;

	Mat temp = Mat::zeros(origin.size().height - 1, origin.size().width - 1, CV_64F);

	int temp_i, temp_j, up_i, up_j;

	if (origin.size().height != origin.size().width)
	{
		cout << "Fail to convert inverse matrix" << endl;

		return output_Result;
	}
	else
	{
		for (int i = 0; i < origin.size().height; i++)
		{
			for (int j = 0; j < origin.size().width; j++)
			{
				for (int k = 0; k < temp.size().height; k++)
				{
					for (int l = 0; l < temp.size().width; l++)
					{
						if (i <= k) up_i = 1;
						else up_i = 0;

						if (j <= l) up_j = 1;
						else up_j = 0;

						temp_i = k + up_i;
						temp_j = l + up_j;

						temp.at<double>(k, l) = origin.at<double>(temp_i, temp_j);
					}
				}

				output_Result.at<double>(i, j) = pow(-1.0, double(i + j + 2)) * Determinant_Matrix(temp);

				//cout << "cofector temp det " << i << " " << j << " : " << Determinant_Matrix(temp) <<endl;

			}
		}

		return output_Result;
	}
}

Mat Inverse_Matrix(Mat origin)
{
	Mat output_Result;

	if (origin.size().height != origin.size().width)
	{
		cout << "Fail to convert inverse matrix" << endl;

		return output_Result;
	}
	else
	{
		output_Result = Transpose_Matrix(Cofector_Matrix(origin)) / Determinant_Matrix(origin);

		return output_Result;
	}
}

Mat Linear_Regression(vector<Point> points, int dimention)
{
	// Let h1(x) = x, h2(x) = x^2, h3(x) = x^3 ......
	// w = (h^(T) * h)^(-1) * (h^(T) * y)

	Mat output_Result;

	Mat matrix_W = Mat::zeros((dimention + 1), 1, CV_64F);

	Mat matrix_Y = Mat::zeros((int)points.size(), 1, CV_64F);

	Mat matrix_H = Mat::zeros((int)points.size(), (dimention + 1), CV_64F);

	//cout << "Max_Degree " << dimention << endl;

	for (int i = 0; i < points.size(); i++)
	{
		for (int j = 0; j <= dimention; j++)
		{
			matrix_H.at<double>(i, j) += pow(points[i].x, (double)j);
		}

		matrix_Y.at<double>(i, 0) = points[i].y;
	}

	for (int i = 0; i < matrix_H.size().height; i++)
	{
		for (int j = 0; j < matrix_H.size().width; j++)
		{
			//cout << "points " << i << " = " << points[i].x <<endl;

			//cout << "matrix_H : " << i << " " << j << " " << matrix_H.at<double>(i, j) << endl;
		}
	}

	matrix_W = Multiple_Matrix(Inverse_Matrix(Multiple_Matrix(Transpose_Matrix(matrix_H), matrix_H)), Multiple_Matrix(Transpose_Matrix(matrix_H), matrix_Y));

	//cout << matrix_W.size().height << " " << matrix_W.size().width <<endl;

	//return output_Result;

	return matrix_W;

}

Mat Show_Graph(Mat origin, Mat coefficient)
{
	Mat output_Result = origin.clone();

	double y;

	for (int i = 1; i < origin.size().width; i++)
	{
		y = 0.0;

		for (int j = 0; j < coefficient.size().height; j++)
		{
			y += coefficient.at<double>(j, 0) * pow((double)i, (double)j);
		}

		if ((int)y >= origin.size().height) y = origin.size().height - 1;
		else if ((int)y <= 0) y = 0;

		output_Result.at<Vec3b>((int)y, i)[0] = 0;
		output_Result.at<Vec3b>((int)y, i)[1] = 255;
		output_Result.at<Vec3b>((int)y, i)[2] = 0;
	}

	return output_Result;
}

Mat RANSAC_Test(vector<Point> dots, int rand_Num, double dist, int p_N)
{

	vector<Point> temp_Line;

	vector<Point> inlier;

	vector<int> temp_Num;

	Mat temp_Line_Coefficient, inlier_Coefficient;

	int c_Num, c_Max = 0;

	double temp_Y;

	srand((unsigned)time(NULL));

	if (dots.size() != 0)
	{

		for (int n = 0; n < p_N; n++)
		{

			c_Num = 0;

			temp_Num.clear();

			temp_Line.clear();

			for (int i = 0; i < rand_Num; i++)
			{

				temp_Num.push_back(rand() % dots.size());

				if (temp_Num.size() > 1)
				{
					for (int j = 0; j < temp_Num.size(); j++)
					{
						if ((temp_Num[j] == temp_Num[temp_Num.size() - 1]) && (j != temp_Num.size() - 1))
						{
							temp_Num.pop_back();
							i -= 1;
						}
					}
				}
			}

			for (int k = 0; k < temp_Num.size(); k++)
			{
				temp_Line.push_back(dots[temp_Num[k]]);
				cout << temp_Num[k] << endl;
			}

			temp_Line_Coefficient = Linear_Regression(temp_Line, 1);

			for (int l = 0; l < dots.size(); l++)
			{
				temp_Y = 0.0;

				for (int m = 0; m < temp_Line_Coefficient.size().height; m++)
				{
					temp_Y += temp_Line_Coefficient.at<double>(m, 0) * pow((double)dots[l].x, (double)m);
				}

				if (abs((double)dots[l].y - temp_Y) <= dist)
				{
					temp_Line.push_back(dots[l]);
					c_Num += 1;
				}

			}

			if (c_Num > c_Max)
			{
				inlier.clear();

				inlier.assign(temp_Line.begin(), temp_Line.end());
			}

			//cout << "Iteration : " << n << endl;
		}

		//cout << "Doen" << endl;

		inlier_Coefficient = Linear_Regression(inlier, 1);

	}

	return inlier_Coefficient;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////







//Set Gray From Depth Mat
Mat Set_Gray_From_Depth(Mat target, Mat Data)
{
	int w = 0;
	int h = 0;

	//target = Mat::zeros(Data.size().height, Data.size().width, CV_8U);

	for (h = 0; h < target.size().height; h++)
	{
		for (w = 0; w < target.size().width; w++)
		{
			if (Data.at<float>(h, w) <= 1500) target.at<uchar>(h, w) = (int)(Data.at<float>(h, w) / 5);
			else target.at<uchar>(h, w) = 0;
		}
	}

	//printf("%d",num);//데이터 갯수 확인
	return target;
}

Mat Set_Gray_All_Full(Mat origin)
{
	Mat result = origin.clone();

	for (int i = 0; i < result.size().height; i++)
	{
		for (int j = 0; j < result.size().width; j++)
		{
			if (result.at<uchar>(i, j) > 0)
				result.at<uchar>(i, j) = 255;
		}
	}

	return result;
}


#define RAD_TO_DEG (45.0/atan((double)1))

struct save_Point
{
	int x_Point;
	int y_Point;
	int Pixel_Value;
	float Real_Value;
};

void q_sort(int numbers[], int left, int right)
{
	int pivot, l_hold, r_hold;
	l_hold = left;
	r_hold = right;
	pivot = numbers[left]; // 0번째 원소를 피봇으로 선택
	while (left < right)
	{
		// 값이 선택한 피봇과 같거나 크다면, 이동할 필요가 없다
		while ((numbers[right] >= pivot) && (left < right))
			right--;

		// 그렇지 않고 값이 피봇보다 작다면,
		// 피봇의 위치에 현재 값을 넣는다.
		if (left != right)
		{
			numbers[left] = numbers[right];
		}
		// 왼쪽부터 현재 위치까지 값을 읽어들이면서
		// 피봇보다 큰 값이 있다면, 값을 이동한다.
		while ((numbers[left] <= pivot) && (left < right))
			left++;
		if (left != right)
		{
			numbers[right] = numbers[left];
			right--;
		}
	}
	// 모든 스캔이 끝났다면, 피봇값을 현재 위치에 입력한다.
	// 이제 피봇을 기준으로 왼쪽에는 피봇보다 작거나 같은 값만 남았다.
	numbers[left] = pivot;
	pivot = left;
	left = l_hold;
	right = r_hold;

	// 재귀호출을 수행한다.
	if (left < pivot)
		q_sort(numbers, left, pivot - 1);
	if (right > pivot)
		q_sort(numbers, pivot + 1, right);
}

void quickSort(int numbers[], int array_size)
{
	q_sort(numbers, 0, array_size - 1);
}

int Partition(float array[], int start_pos, int end_pos)

{

	float pivot = array[start_pos];       // Smaller than pivot on left; larger on right

	int left_index = start_pos;    // First element

	int right_index = end_pos; // Last element

	while (1) // Loop forever; return once partitioning is completed

	{

		// Skip over large elements on right

		while (array[right_index] > pivot && right_index >= start_pos)

			right_index--;

		// Skip over small elements on left

		while (array[left_index] < pivot && left_index <= end_pos)

			left_index++;

		if (left_index < right_index)          // Exchange if halves aren't complete

		{

			float temp = array[left_index];

			array[left_index] = array[right_index];

			array[right_index] = temp;

			left_index++;                         // Skip over exchanged values

			right_index--;

		}

		else                                            // Otherwise, return location of pivot

			return  right_index;

	}

}

void QuickSort_Float(float array[], int start_pos, int end_pos)

{

	if (start_pos == end_pos) // Only one element

		return;

	int middle_pos = Partition(array, start_pos, end_pos);  // Reposition elements

	QuickSort_Float(array, start_pos, middle_pos);          // Sort left half

	QuickSort_Float(array, middle_pos + 1, end_pos);     // Sort right half

}

int Partition_Point2d(vector<Point2d> &point, int start_pos, int end_pos)

{

	double pivot = point[start_pos].x;       // Smaller than pivot on left; larger on right

	int left_index = start_pos;    // First element

	int right_index = end_pos; // Last element

	while (1) // Loop forever; return once partitioning is completed

	{

		// Skip over large elements on right

		while (point[right_index].x > pivot && right_index >= start_pos)

			right_index--;

		// Skip over small elements on left

		while (point[left_index].x < pivot && left_index <= end_pos)

			left_index++;

		if (left_index < right_index)          // Exchange if halves aren't complete

		{

			Point2d temp = point[left_index];

			point[left_index] = point[right_index];

			point[right_index] = temp;

			left_index++;                         // Skip over exchanged values

			right_index--;

		}

		else                                            // Otherwise, return location of pivot

			return  right_index;

	}

}

void QuickSort_Point2d(vector<Point2d> &point, int start_pos, int end_pos)

{

	if (start_pos == end_pos) // Only one element

		return;

	int middle_pos = Partition_Point2d(point, start_pos, end_pos);  // Reposition elements

	QuickSort_Point2d(point, start_pos, middle_pos);          // Sort left half

	QuickSort_Point2d(point, middle_pos + 1, end_pos);     // Sort right half

}

int Partition_Point(vector<Point> &point, int start_pos, int end_pos)

{

	double pivot = point[start_pos].x;       // Smaller than pivot on left; larger on right

	int left_index = start_pos;    // First element

	int right_index = end_pos; // Last element

	while (1) // Loop forever; return once partitioning is completed

	{

		// Skip over large elements on right

		while (point[right_index].x > pivot && right_index >= start_pos)

			right_index--;

		// Skip over small elements on left

		while (point[left_index].x < pivot && left_index <= end_pos)

			left_index++;

		if (left_index < right_index)          // Exchange if halves aren't complete

		{

			Point2d temp = point[left_index];

			point[left_index] = point[right_index];

			point[right_index] = temp;

			left_index++;                         // Skip over exchanged values

			right_index--;

		}

		else                                            // Otherwise, return location of pivot

			return  right_index;

	}

}

void QuickSort_Point(vector<Point> &point, int start_pos, int end_pos)

{

	if (start_pos == end_pos) // Only one element

		return;

	int middle_pos = Partition_Point(point, start_pos, end_pos);  // Reposition elements

	QuickSort_Point(point, start_pos, middle_pos);          // Sort left half

	QuickSort_Point(point, middle_pos + 1, end_pos);     // Sort right half

}

int Partition_PointY(vector<Point> &point, int start_pos, int end_pos)

{

	double pivot = point[start_pos].y;       // Smaller than pivot on left; larger on right

	int left_index = start_pos;    // First element

	int right_index = end_pos; // Last element

	while (1) // Loop forever; return once partitioning is completed

	{

		// Skip over large elements on right

		while (point[right_index].y > pivot && right_index >= start_pos)

			right_index--;

		// Skip over small elements on left

		while (point[left_index].y < pivot && left_index <= end_pos)

			left_index++;

		if (left_index < right_index)          // Exchange if halves aren't complete

		{

			Point2d temp = point[left_index];

			point[left_index] = point[right_index];

			point[right_index] = temp;

			left_index++;                         // Skip over exchanged values

			right_index--;

		}

		else                                            // Otherwise, return location of pivot

			return  right_index;

	}

}

void QuickSort_PointY(vector<Point> &point, int start_pos, int end_pos)

{

	if (start_pos == end_pos) // Only one element

		return;

	int middle_pos = Partition_PointY(point, start_pos, end_pos);  // Reposition elements

	QuickSort_PointY(point, start_pos, middle_pos);          // Sort left half

	QuickSort_PointY(point, middle_pos + 1, end_pos);     // Sort right half

}

save_Point Find_Maximum_Pixel(Mat origin, Mat source)
{
	save_Point m_Point, temp_Point;

	m_Point.x_Point = 0;

	m_Point.y_Point = 0;

	m_Point.Pixel_Value = 0;

	for (int i = 0; i < origin.size().height; i++)
	{
		for (int j = 0; j < origin.size().width; j++)
		{
			if ((origin.at<uchar>(i, j) > m_Point.Pixel_Value) && (source.at<uchar>(i, j) != 0))
			{
				m_Point.x_Point = j;

				m_Point.y_Point = i;

				m_Point.Pixel_Value = origin.at<uchar>(i, j);
			}
		}
	}

	return m_Point;
}

Mat Odd_Median(Mat origin, int Odd)
{
	int *sort_number = new int[Odd*Odd];

	int even_val[4] = { 0, 0, 0, 0 };

	int num = 0, total_num = 0;

	int w = 0, h = 0, i = 0, j = 0, k = 0;
	int median_Value = 0;
	//Mat copy = origin.clone();
	Mat copy = Mat::zeros(origin.size(), CV_32F);

	for (h = 0; h < origin.size().height; h++)
	{
		for (w = 0; w < origin.size().width; w++)
		{
			num = 0;
			for (i = 0; i < Odd; i++)
			{
				for (j = 0; j < Odd; j++)
				{
					if ((((h - 1) + i) < 0) || (((h - 1) + i) >= origin.size().height) || (((w - 1) + j) < 0) || (((w - 1) + j) >= origin.size().width))
					{
						sort_number[num] = 0;
					}
					else
					{
						sort_number[num] = origin.at<float>((h - 1) + i, (w - 1) + j);
					}
					num++;
				}
			}

			quickSort(sort_number, Odd*Odd);

			copy.at<float>(h, w) = sort_number[((Odd*Odd) / 2) + 1];

			//cout << ((Odd*Odd)/2) + 1 << copy.at<float>(h,w) <<endl;

			total_num++;

		}
	}

	//cout << total_num <<endl;

	delete[] sort_number;

	return copy;
}

Mat even_Median(Mat odd_median)
{
	//Mat copy = odd_median.clone();
	Mat copy = Mat::zeros(odd_median.size(), CV_32F);

	int h = 0, w = 0;
	int i = 0, j = 0;

	int total_num = 0, num = 0;

	int even_val[4] = { 0, 0, 0, 0 };

	for (h = 0; h < copy.size().height - 1; h++)
	{
		for (w = 0; w < copy.size().width - 1; w++)
		{
			num = 0;
			for (i = 0; i < 2; i++)
			{
				for (j = 0; j < 2; j++)
				{
					even_val[num] = odd_median.at<float>(h + i, w + j);
					num++;
				}
			}

			quickSort(even_val, 4);

			copy.at<float>(h, w) = (even_val[1] + even_val[2]) / 2;

			//cout << ((Odd*Odd)/2) + 1 << copy.at<float>(h,w) <<endl;

			total_num++;
		}

	}

	return copy;

}

Mat Sobel_Mask_Depth(Mat origin, Mat angle)
{
	//Mat copy = origin.clone();
	Mat copy = Mat::zeros(origin.size(), CV_32F);

	double degree = 0.0;

	int h = 0, w = 0, k = 0, l = 0;
	float h_Value = 0, w_Value = 0, total_Value = 0;;

	static int Sobel_mask1[3][3] = {
		{ -1, -2, -1 },
		{ 0, 0, 0 },
		{ 1, 2, 1 } };
	static int Sobel_mask2[3][3] = {
		{ -1, 0, 1 },
		{ -2, 0, 2 },
		{ -1, 0, 1 } };

	for (h = 0; h < origin.size().height; h++)
	{
		for (w = 0; w < origin.size().width; w++)
		{

			h_Value = 0;
			w_Value = 0;
			for (k = -1; k < 2; k++)
			{
				for (l = -1; l < 2; l++)
				{
					if ((h + k < 0) || (h + k >= origin.size().height) || (w + l < 0) || (w + l >= origin.size().width))
					{
						h_Value += 0;
						w_Value += 0;
					}
					else
					{
						h_Value += origin.at<float>(h + k, w + l) * Sobel_mask1[k + 1][l + 1];
						w_Value += origin.at<float>(h + k, w + l) * Sobel_mask2[k + 1][l + 1];
					}
				}
			}

			degree = RAD_TO_DEG*atan2((double)(h_Value), (double)(w_Value));

			if (degree < 0)
			{

				degree = -degree;

				if ((degree < 22.5) && (degree > 157.5))
				{
					angle.at<uchar>(h, w) = 1;
				}
				else if ((degree < 67.5) && (degree >= 22.5))
				{
					angle.at<uchar>(h, w) = 2;
				}
				else if ((degree < 112.5) && (degree >= 67.5))
				{
					angle.at<uchar>(h, w) = 3;
				}
				else if ((degree < 157.5) && (degree >= 112.5))
				{
					angle.at<uchar>(h, w) = 4;
				}



			}

			copy.at<float>(h, w) = (int)sqrtf((h_Value*h_Value) + (w_Value*w_Value));

		}


	}

	return copy;
}

Mat Sobel_Mask_Gray(Mat origin, Mat angle)
{

	//FILE* fp;
	//fp = fopen("log.txt", "wt");

	//Mat copy = origin.clone();
	Mat copy = Mat::zeros(origin.size(), CV_8U);

	int h = 0, w = 0, k = 0, l = 0;
	int h_Value = 0, w_Value = 0, total_Value = 0;;
	double degree = 0;

	static int Sobel_mask1[3][3] = {
		{ -1, -2, -1 },
		{ 0, 0, 0 },
		{ 1, 2, 1 } };
	static int Sobel_mask2[3][3] = {
		{ -1, 0, 1 },
		{ -2, 0, 2 },
		{ -1, 0, 1 } };

	for (h = 0; h < origin.size().height; h++)
	{
		for (w = 0; w < origin.size().width; w++)
		{

			h_Value = 0;
			w_Value = 0;
			for (k = -1; k < 2; k++)
			{
				for (l = -1; l < 2; l++)
				{
					if ((h + k < 0) || (h + k >= origin.size().height) || (w + l < 0) || (w + l >= origin.size().width))
					{
						h_Value += 0;
						w_Value += 0;
					}
					else
					{
						h_Value += origin.at<uchar>(h + k, w + l) * Sobel_mask1[k + 1][l + 1];
						w_Value += origin.at<uchar>(h + k, w + l) * Sobel_mask2[k + 1][l + 1];
					}
				}
			}

			total_Value = (int)sqrtf((h_Value*h_Value) + (w_Value*w_Value));

			if ((w_Value != 0) && (h_Value != 0)) degree = RAD_TO_DEG*atan2((double)(h_Value), (double)(w_Value));
			else if ((w_Value == 0) && (h_Value != 0)) degree = 90;
			else degree = 0;

			if (degree < 0)
			{
				degree = degree + 180;

			}

			if (degree >= 0)
			{

				if ((degree < 22.5) || (degree >= 157.5))
				{
					angle.at<uchar>(h, w) = 1;
				}
				else if ((degree < 67.5) && (degree >= 22.5))
				{
					angle.at<uchar>(h, w) = 2;
				}
				else if ((degree < 112.5) && (degree >= 67.5))
				{
					angle.at<uchar>(h, w) = 3;
				}
				else if ((degree < 157.5) && (degree >= 112.5))
				{
					angle.at<uchar>(h, w) = 4;
				}
				else if (degree == 0)
				{
					angle.at<uchar>(h, w) = 0;
				}

			}

			if (total_Value > 255) total_Value = 255;

			copy.at<uchar>(h, w) = total_Value;

		}


	}

	//fclose(fp);

	return copy;
}

Mat Square_Average_Interpolation(Mat origin)
{
	int h = 0, w = 0, x = 0, y = 0;
	int i = 0, j = 0, k = 0, l = 0, m = 0;
	int num = 0, count = 0, sum = 0, value = 0;
	Mat copy = origin.clone();
	//Mat copy = Mat::zeros(origin.size(), CV_32F);

	for (h = 0; h < origin.size().height; h++)
	{
		//printf("%d \n",h);
		for (w = 0; w < origin.size().width; w++)
		{
			//printf("width plus \n");
			if ((int)origin.at<float>(h, w) == 0)
			{
				//cout << "Found 0 at " << w << ", " << h << "\n" <<endl;
				count = 0;
				sum = 0;
				value = 0;
				//printf("after break \n");

				for (x = 1; x <= origin.size().width; x++)
				{


					for (i = (h - x); i <= (h + (x - 1)); i++)
					{
						if ((i >= 0) && ((w - x) >= 0) && (i < origin.size().height) && ((w - x) < origin.size().width))
						{
							if ((int)origin.at<float>(i, w - x) != 0)
							{
								count += 1;
								sum += (int)origin.at<float>(i, w - x);
							}
						}
					}

					for (j = (w - x); j <= (w + (x - 1)); j++)
					{
						if ((j >= 0) && ((h + x) >= 0) && ((h + x) < origin.size().height) && (j < origin.size().width))
						{
							if ((int)origin.at<float>(h + x, j) != 0)
							{
								count += 1;
								sum += (int)origin.at<float>(h + x, j);
							}
						}
					}
					for (k = (h - (x + 1)); k <= (h + x); k++)
					{
						if ((k >= 0) && ((w + x) >= 0) && (k < origin.size().height) && ((w + x) < origin.size().width))
						{
							if ((int)origin.at<float>(k, w + x) != 0)
							{
								count += 1;
								sum += (int)origin.at<float>(k, w + x);
							}
						}
					}
					for (l = (w - (x + 1)); l <= (w + x); l++)
					{
						if ((l >= 0) && ((h - x) >= 0) && ((h - x) < origin.size().height) && (l < origin.size().width))
						{
							if ((int)origin.at<float>(h - x, l) != 0)
							{
								count += 1;
								sum += (int)origin.at<float>(h - x, l);
							}
						}
					}
					if (sum != 0)
					{
						value = sum / count;
						copy.at<float>(h, w) = value;
						//cout << copy.at<float>(h,w) <<endl;
						//printf("before break \n");
						break;
					}
				}
				//printf("done1 %d %d\n",w,h);
			}
			//else continue;

			num++;
		}
	}
	//cout << num <<endl;

	return copy;
}

Mat Nearest_Neighbor_Interpolation(Mat origin)
{
	int h = 0, w = 0, x = 0, y = 0;
	int i = 0, j = 0, k = 0, l = 0, m = 0;
	int num = 0, count = 0, sum = 0, value = 0;
	int distance = 0;
	Mat copy = origin.clone();
	//Mat copy = Mat::zeros(origin.size(), CV_32F);

	for (h = 0; h < origin.size().height; h++)
	{
		//printf("%d \n",h);
		for (w = 0; w < origin.size().width; w++)
		{
			//printf("width plus \n");
			if ((int)origin.at<float>(h, w) == 0)
			{
				//cout << "Found 0 at " << w << ", " << h << "\n" <<endl;
				count = 0;
				sum = 0;
				value = 0;
				//printf("after break \n");

				for (x = 1; x <= origin.size().width; x++)
				{
					distance = 0;
					for (y = 0; y <= x; y++)
					{
						if (((h + x) < origin.size().height) || ((h + y) < origin.size().height) || ((w + x) < origin.size().width) || ((w + y) < origin.size().width) || ((h - x) >= 0) || ((h - y) >= 0) || ((w - x) >= 0) || ((w - y) >= 0))
						{
							if (((h + y) < origin.size().height) && ((w + x) < origin.size().width))
							{
								if ((int)origin.at<float>(h + y, w + x) != 0)
								{
									count += 1;
									sum += (int)origin.at<float>(h + y, w + x);
								}
							}
							if (((h + y) < origin.size().height) && ((w - x) >= 0))
							{
								if ((int)origin.at<float>(h + y, (w - x)) != 0)
								{
									count += 1;
									sum += (int)origin.at<float>(h + y, (w - x));
								}
							}
							if (((h + x) < origin.size().height) && ((w + y) < origin.size().width))
							{
								if ((int)origin.at<float>(h + x, w + y) != 0)
								{
									count += 1;
									sum += (int)origin.at<float>(h + x, w + y);
								}
							}
							if (((h + x) < origin.size().height) && ((w - y) >= 0))
							{
								if ((int)origin.at<float>(h + x, (w - y)) != 0)
								{
									count += 1;
									sum += (int)origin.at<float>(h + x, (w - y));
								}
							}
							if (((h - y) >= 0) && ((w + x) < origin.size().width))
							{
								if ((int)origin.at<float>((h - y), w + x) != 0)
								{
									count += 1;
									sum += (int)origin.at<float>((h - y), w + x);
								}
							}
							if (((h - y) >= 0) && ((w - x) >= 0))
							{
								if ((int)origin.at<float>((h - y), (w - x)) != 0)
								{
									count += 1;
									sum += (int)origin.at<float>((h - y), (w - x));
								}
							}
							if (((h - x) >= 0) && ((w + y) < origin.size().width))
							{
								if ((int)origin.at<float>((h - x), w + y) != 0)
								{
									count += 1;
									sum += (int)origin.at<float>((h - x), w + y);
								}
							}
							if (((h - x) >= 0) && ((w - y) >= 0))
							{
								if ((int)origin.at<float>((h - x), (w - y)) != 0)
								{
									count += 1;
									sum += (int)origin.at<float>((h - x), (w - y));
								}
							}
							if (sum != 0) break;
						}
					}
					if (sum != 0)
					{
						value = sum / count;
						copy.at<float>(h, w) = value;
						//cout << copy.at<float>(h,w) <<endl;
						//printf("before break \n");
						break;
					}
					//printf("done1 %d %d\n",w,h);
				}

			}
			//else continue;

			num++;
		}
	}
	//cout << num <<endl;

	return copy;
}

Mat Non_Maximum_Suppression_Gray(Mat origin, Mat angle, int thres_Value)
{
	//Mat copy = origin.clone();
	Mat copy = Mat::zeros(origin.size(), CV_8U);

	int Find_Maximum[5] = { 0, 0, 0, 0, 0 };

	int w = 0, h = 0, i = 0;

	int p_Value = 0;

	for (h = 0; h < origin.size().height; h++)
	{
		for (w = 0; w < origin.size().width; w++)
		{

			if ((w - 1 >= 0) && (h - 1 >= 0) && (w + 1 < origin.size().width) && (h + 1 < origin.size().height))
			{

				if (angle.at<uchar>(h, w) == 1)
				{

					if ((origin.at<uchar>(h, w) >= origin.at<uchar>(h, w - 1)) && (origin.at<uchar>(h, w) >= origin.at<uchar>(h, w + 1)))
					{
						copy.at<uchar>(h, w) = origin.at<uchar>(h, w);
					}
					else
					{
						copy.at<uchar>(h, w) = 0;
					}


				}
				else if (angle.at<uchar>(h, w) == 2)
				{
					if ((origin.at<uchar>(h, w) >= origin.at<uchar>(h - 1, w + 1)) && (origin.at<uchar>(h, w) >= origin.at<uchar>(h + 1, w - 1)))
					{
						copy.at<uchar>(h, w) = origin.at<uchar>(h, w);
					}
					else
					{
						copy.at<uchar>(h, w) = 0;
					}

				}
				else if (angle.at<uchar>(h, w) == 3)
				{
					if ((origin.at<uchar>(h, w) >= origin.at<uchar>(h - 1, w)) && (origin.at<uchar>(h, w) >= origin.at<uchar>(h + 1, w)))
					{
						copy.at<uchar>(h, w) = origin.at<uchar>(h, w);
					}
					else
					{
						copy.at<uchar>(h, w) = 0;
					}

				}
				else if (angle.at<uchar>(h, w) == 4)
				{
					if ((origin.at<uchar>(h, w) >= origin.at<uchar>(h - 1, w - 1)) && (origin.at<uchar>(h, w) >= origin.at<uchar>(h + 1, w + 1)))
					{
						copy.at<uchar>(h, w) = origin.at<uchar>(h, w);
					}
					else
					{
						copy.at<uchar>(h, w) = 0;
					}

				}
				else if (angle.at<uchar>(h, w) == 0)
				{
					Find_Maximum[2] = origin.at<uchar>(h, w);

					if ((h - 1) < 0)
					{
						Find_Maximum[0] = 0;
					}
					else
					{
						Find_Maximum[0] = origin.at<uchar>(h - 1, w);
					}
					if ((w - 1) < 0)
					{
						Find_Maximum[1] = 0;
					}
					else
					{
						Find_Maximum[1] = origin.at<uchar>(h, w - 1);
					}
					if ((w + 1) >= origin.size().width)
					{
						Find_Maximum[3] = 0;
					}
					else
					{
						Find_Maximum[3] = origin.at<uchar>(h, w + 1);
					}
					if ((h + 1) >= origin.size().height)
					{
						Find_Maximum[4] = 0;
					}
					else
					{
						Find_Maximum[4] = origin.at<uchar>(h + 1, w);
					}

					quickSort(Find_Maximum, 5);

					if (origin.at<uchar>(h, w) == Find_Maximum[4])
					{
						copy.at<uchar>(h, w) = Find_Maximum[4];
					}
					else
					{
						copy.at<uchar>(h, w) = 0;
					}
				}

			}
			/*if(origin.at<uchar>(h, w) >= thres_Value)
			{
			copy.at<uchar>(h, w) = origin.at<uchar>(h, w);
			}*/



		}

	}

	return copy;
}

Mat Double_Thresholding(Mat origin, int low_Thres, int high_Thres)
{
	//Mat copy = origin.clone();
	Mat copy = Mat::zeros(origin.size(), CV_8U);

	int w = 0, h = 0;

	for (h = 0; h < origin.size().height; h++)
	{
		for (w = 0; w < origin.size().width; w++)
		{
			if (origin.at<uchar>(h, w) < low_Thres)  copy.at<uchar>(h, w) = 0;
			else if ((origin.at<uchar>(h, w) >= low_Thres) && (origin.at<uchar>(h, w) < high_Thres))  copy.at<uchar>(h, w) = 30;
			else if (origin.at<uchar>(h, w) >= high_Thres) copy.at<uchar>(h, w) = 255;

			//cout << (int)copy.at<uchar>(h, w) <<endl;
		}
	}

	return copy;
}

void Recursive_For_Hyteresis(Mat origin, int w, int h, int low_Thres, int high_Thres, vector<save_Point> &vec_sv_Point, int &flag_Strong)
{
	int i = 0, j = 0, k = 0;

	save_Point temp_Save;

	for (i = -1; i <= 1; i++)
	{
		for (j = -1; j <= 1; j++)
		{
			if (((h + i) >= 0) && ((h + i) < origin.size().height) && ((w + j) >= 0) && ((w + j) < origin.size().width))
			{
				if (origin.at<uchar>((h + i), (w + j)) == low_Thres)
				{
					//if(temp_flag == 1) flag_Strong = 1;
					origin.at<uchar>((h + i), (w + j)) = 0;
					temp_Save.x_Point = w + j;
					temp_Save.y_Point = h + i;
					vec_sv_Point.push_back(temp_Save);
					Recursive_For_Hyteresis(origin, (w + j), (h + i), low_Thres, high_Thres, vec_sv_Point, flag_Strong);
				}
				else if (origin.at<uchar>((h + i), (w + j)) == high_Thres)
				{
					//temp_flag = 1;
					flag_Strong = 1;
					/*origin.at<uchar>((h + i), (w + j)) = 0;
					temp_Save.x_Point = w + j;
					temp_Save.y_Point = h + i;
					vec_sv_Point.push_back(temp_Save);
					Recursive_For_Hyteresis(origin, (w + j), (h + i), low_Thres, high_Thres, vec_sv_Point, flag_Strong);*/
				}

			}

		}
	}


}

Mat Hysteresis_Edge_Tracking_Recursive(Mat origin, int low_Thres, int high_Thres)
{
	//Mat copy = Mat::zeros(origin.size(), CV_8U);
	Mat copy = origin.clone();

	int h = 0, w = 0, k = 0;

	vector<save_Point> vec_sv_Point;

	save_Point temp_Save;

	int flag_Strong = 0;

	for (h = 0; h < origin.size().height; h++)
	{

		for (w = 0; w < origin.size().width; w++)
		{
			if (origin.at<uchar>(h, w) == low_Thres)
			{
				flag_Strong = 0;
				copy.at<uchar>(h, w) = 0;
				temp_Save.x_Point = w;
				temp_Save.y_Point = h;
				vec_sv_Point.push_back(temp_Save);
				Recursive_For_Hyteresis(copy, w, h, low_Thres, high_Thres, vec_sv_Point, flag_Strong);

				//cout << "vec : " << vec_sv_Point.size() <<endl;
				//cout << flag_Strong <<endl;

				if (flag_Strong == 1)
				{

					for (k = 0; k < vec_sv_Point.size(); k++)
					{
						copy.at<uchar>(vec_sv_Point[k].y_Point, vec_sv_Point[k].x_Point) = 255;

						//vec_sv_Point.pop_back();
					}
					vec_sv_Point.~vector();
					flag_Strong = 0;
				}
				else if (flag_Strong == 0)
				{
					//cout << "find 0" << endl;

					for (k = 0; k < vec_sv_Point.size(); k++)
					{
						copy.at<uchar>(vec_sv_Point[k].y_Point, vec_sv_Point[k].x_Point) = 0;

						//vec_sv_Point.pop_back();
					}
					vec_sv_Point.~vector();
					flag_Strong = 0;
				}
			}

		}

	}

	return copy;
}

Mat Gaussian_Filter(Mat origin, float sigma, int filter_Size)
{
	Mat copy = Mat::zeros(origin.size(), CV_8U);
	Mat temp_Mat = Mat::zeros(origin.size(), CV_32F);

	float equ_Front, equ_Back, temp_Back;
	float temp_X_Total = 0.0f, temp_Y_Total = 0.0f;
	vector<float> g_value;
	int i = 0, j = 0;
	int h = 0, w = 0;

	//int filter_Size = 4*sigma + 1;

	g_value.resize(filter_Size);

	//sigma = 0.3f*((filter_Size*filter_Size)/2 - 1) + 0.8f;

	equ_Front = 1.0f / (sqrt(2.0f * 3.1415926535f) * sigma);
	equ_Back = 2.0f * sigma * sigma;

	for (i = 0; i <= (filter_Size / 2); i++)
	{
		temp_Back = -(((i - (filter_Size / 2)) * (i - (filter_Size / 2))) / equ_Back);

		g_value[i] = equ_Front * exp(temp_Back);
		g_value[filter_Size - 1 - i] = equ_Front * exp(temp_Back);

		//cout<< (i - (filter_Size/2)) << "   " << g_value[i] << "   " << filter_Size-1-i << "   " << g_value[filter_Size-1-i] <<endl;
	}

	for (h = 0; h < origin.size().height; h++)
	{

		for (w = 0; w < origin.size().width; w++)
		{
			temp_X_Total = 0.0f;


			for (j = 0; j < filter_Size; j++)
			{
				if (((w - (j - filter_Size / 2)) < 0) || ((w - (j - filter_Size / 2)) >= origin.size().width))
				{
					temp_X_Total += 0;
				}
				else
				{
					temp_X_Total += g_value[j] * origin.at<uchar>(h, (w - (j - filter_Size / 2)));
				}
			}

			temp_Mat.at<float>(h, w) = temp_X_Total;

			//cout << temp_Mat.at<float>(h, w) <<endl;

		}

	}

	for (h = 0; h < origin.size().height; h++)
	{

		for (w = 0; w < origin.size().width; w++)
		{
			temp_Y_Total = 0.0f;

			for (j = 0; j < filter_Size; j++)
			{
				if (((h - (j - filter_Size / 2)) < 0) || ((h - (j - filter_Size / 2)) >= origin.size().height))
				{
					temp_Y_Total += 0;
				}
				else
				{
					temp_Y_Total += g_value[j] * temp_Mat.at<float>((h - (j - filter_Size / 2)), w);
				}
			}

			//cout << (int)temp_Y_Total <<endl;

			copy.at<uchar>(h, w) = temp_Y_Total;
		}

	}

	//cout << "done" <<endl;

	return copy;
}

Mat Reverse_Gray(Mat origin)
{
	Mat copy = Mat::zeros(origin.size(), CV_8U);

	int h, w;

	for (h = 0; h < origin.size().height; h++)
	{

		for (w = 0; w < origin.size().width; w++)
		{
			if (origin.at<uchar>(h, w) == 255) copy.at<uchar>(h, w) = 0;
			else copy.at<uchar>(h, w) = 255;
		}

	}

	return copy;
}

Mat Chamfer_Distance_Transform_Gray(Mat origin)
{
	Mat copy = origin.clone();

	Mat mask(3, 3, CV_32F);

	float numbers[5];

	int i, j;

	int h = 0, w = 0;

	for (i = -1; i <= 1; i++)
	{
		for (j = -1; j <= 1; j++)
		{
			//mask.at<float>(i + 1, j + 1) = sqrt((double)(i*i + j*j)) * 7.0 ;
			mask.at<float>(i + 1, j + 1) = sqrt((double)(i*i + j*j)) * 5.0;
			//cout << (int)mask.at<uchar>(i + 1, j + 1) << "	";
		}
		//cout <<endl;
	}

	for (h = 1; h < origin.size().height - 1; h++)
	{

		for (w = 1; w < origin.size().width - 1; w++)
		{
			/*if((h - 1) < 0)
			{
			numbers[0] = 255;
			numbers[1] = 255;
			numbers[2] = 255;
			}
			if((w - 1) < 0)
			{
			numbers[0] = 255;
			numbers[3] = 255;
			}
			if((w + 1) >= image_Width)
			{
			numbers[2] = 255;
			}
			if(((h - 1) >= 0)&&((w - 1) >= 0)&&((w + 1) < image_Width))
			{*/
			numbers[0] = (float)copy.at<uchar>(h - 1, w - 1) + mask.at<float>(0, 0);
			numbers[1] = (float)copy.at<uchar>(h - 1, w) + mask.at<float>(0, 1);
			numbers[2] = (float)copy.at<uchar>(h - 1, w + 1) + mask.at<float>(0, 2);
			numbers[3] = (float)copy.at<uchar>(h, w - 1) + mask.at<float>(1, 0);
			numbers[4] = (float)copy.at<uchar>(h, w) + mask.at<float>(1, 1);
			//}

			QuickSort_Float(numbers, 0, 4);

			if ((int)numbers[0] >= 255) numbers[0] = 255.0;

			copy.at<uchar>(h, w) = (int)numbers[0];

			//cout << numbers[0] <<endl;

		}

	}

	for (h = (origin.size().height - 2); h > 0; h--)
	{

		for (w = (origin.size().width - 2); w > 0; w--)
		{
			/*if((h + 1) >= image_Height)
			{
			numbers[2] = 255;
			numbers[3] = 255;
			numbers[4] = 255;
			}
			if((w + 1) >= image_Width)
			{
			numbers[1] = 255;
			numbers[4] = 255;
			}
			if((w - 1) < 0)
			{
			numbers[2] = 255;
			}
			if(((h-1) >= 0)&&((w-1) >= 0)&&((w + 1) < image_Width))
			{*/
			numbers[0] = (float)copy.at<uchar>(h, w) + mask.at<float>(1, 1);
			numbers[1] = (float)copy.at<uchar>(h, w + 1) + mask.at<float>(1, 2);
			numbers[2] = (float)copy.at<uchar>(h + 1, w - 1) + mask.at<float>(2, 0);
			numbers[3] = (float)copy.at<uchar>(h + 1, w) + mask.at<float>(2, 1);
			numbers[4] = (float)copy.at<uchar>(h + 1, w + 1) + mask.at<float>(2, 2);
			//}

			QuickSort_Float(numbers, 0, 4);

			if ((int)numbers[0] >= 255) numbers[0] = 255.0;

			copy.at<uchar>(h, w) = (int)numbers[0];
		}

	}

	return copy;
}

Mat Resize_Image_Gray(Mat origin, int numerator, int denominator)
{
	Mat copy = Mat::zeros((int)origin.size().height*numerator / denominator, (int)origin.size().width*numerator / denominator, CV_8U);

	int h = 0, w = 0, h_copy = 0, w_copy = 0;

	int i = 0, j = 0;

	if (numerator <= denominator)
	{
		for (h = 0; h < origin.size().height; h += denominator)
		{
			for (i = 0; i < numerator; i++)
			{
				w_copy = 0;
				for (w = 0; w < origin.size().width; w += denominator)
				{
					for (j = 0; j < numerator; j++)
					{
						if ((h_copy < copy.size().height) && (w_copy < copy.size().width))
						{
							copy.at<uchar>(h_copy, w_copy) = origin.at<uchar>(h + i, w + j);
						}

						w_copy++;
					}

				}
				h_copy++;
			}
		}
	}
	else
	{
		for (h = 0; h < origin.size().height*numerator / denominator; h += numerator)
		{
			for (i = 0; i < denominator; i++)
			{
				w_copy = 0;
				for (w = 0; w < origin.size().width*numerator / denominator; w += numerator)
				{
					for (j = 0; j < denominator; j++)
					{
						copy.at<uchar>(h + i, w + j) = origin.at<uchar>(h_copy, w_copy);

						w_copy++;
					}

				}
				h_copy++;
			}
		}
	}
	//cout << "done" <<endl;

	//imshow("test",copy);

	return copy;
}

void Check_Value_Gray(Mat origin, string filename)
{
	FILE* pFile;
	string str;

	pFile = fopen(filename.c_str(), "w");

	for (int i = 0; i < origin.size().height; i++)
	{
		for (int j = 0; j < origin.size().width; j++)
		{
			str = format("%d	", origin.at<uchar>(i, j));
			fputs(str.c_str(), pFile);
		}
		fputs("\n", pFile);
	}

	fclose(pFile);
}

void Recursive_For_Only_Edge_Tracking(Mat origin, int w, int h, vector<save_Point> &vec_sv_Point)
{
	int i = 0, j = 0, k = 0;

	save_Point temp_Save;

	for (i = -1; i <= 1; i++)
	{
		for (j = -1; j <= 1; j++)
		{
			if (((h + i) >= 0) && ((h + i) < origin.size().height) && ((w + j) >= 0) && ((w + j) < origin.size().width))
			{
				if (origin.at<uchar>((h + i), (w + j)) == 255)
				{
					//if(temp_flag == 1) flag_Strong = 1;
					origin.at<uchar>((h + i), (w + j)) = 0;
					temp_Save.x_Point = w + j;
					temp_Save.y_Point = h + i;
					vec_sv_Point.push_back(temp_Save);
					Recursive_For_Only_Edge_Tracking(origin, (w + j), (h + i), vec_sv_Point);
				}
				//else break;

			}

		}
	}


}

vector<save_Point> Edge_Tracking(Mat origin, Point wrist)
{
	Mat copy = origin.clone();

	vector<save_Point> result;

	save_Point temp, find;

	int range = 7;

	///*
	for (int i = -range; i <= range; i++)
	{
		for (int j = -range; j <= range; j++)
		{

			temp.x_Point = wrist.x * 2 + j;
			temp.y_Point = wrist.y * 2 + i;

			if (temp.x_Point < 0) temp.x_Point = 0;
			else if (temp.x_Point >= copy.size().width) temp.x_Point = copy.size().width - 1;

			if (temp.y_Point < 0) temp.y_Point = 0;
			else if (temp.y_Point >= copy.size().height) temp.y_Point = copy.size().height - 1;

			//result.push_back(temp);

			if (copy.at<uchar>(temp.y_Point, temp.x_Point) == 255)
			{

				copy.at<uchar>(temp.y_Point, temp.x_Point) = 0;

				result.push_back(temp);

				find.x_Point = temp.x_Point;

				find.y_Point = temp.y_Point;


			}
		}
	}
	//*/

	if (result.size() != 0) Recursive_For_Only_Edge_Tracking(copy, find.x_Point, find.y_Point, result);

	//Recursive_For_Only_Edge_Tracking(copy, wrist.x, wrist.y, result);

	//if (result.size() != 0) imshow("Check", copy);

	return result;
}

void Recursive_For_Only_Edge_Tracking_Graph(Mat origin, int w, int h, vector<Point> &vec_sv_Point)
{
	int i = 0, j = 0, k = 0;

	Point temp_Save;

	double min_Value = 0.0;

	for (i = -2; i <= 2; i++)
	{
		for (j = -2; j <= 2; j++)
		{
			if (((h + i) >= 0) && ((h + i) < origin.size().height) && ((w + j) >= 0) && ((w + j) < origin.size().width))
			{
				if (origin.at<Vec3b>((h + i), (w + j))[2] == 255)
				{
					//if(temp_flag == 1) flag_Strong = 1;
					origin.at<Vec3b>((h + i), (w + j))[2] = 0;
					temp_Save.x = w + j;
					temp_Save.y = h + i;
					vec_sv_Point.push_back(temp_Save);
					Recursive_For_Only_Edge_Tracking_Graph(origin, (w + j), (h + i), vec_sv_Point);
				}
				//else break;

			}

		}
	}


}

vector<Point> Edge_Tracking_Graph(Mat origin)
{
	Mat copy = origin.clone();

	vector<Point> result;

	Point temp, find;

	int range = 7;

	int j = 0;

	find.x = 0;
	find.y = 0;
	///*
	for (int i = 0; i < copy.size().height; i++)
	{

		if (copy.at<Vec3b>(i, j)[2] == 255)
		{

			copy.at<Vec3b>(i, j)[2] = 0;

			result.push_back(temp);

			find.x = j;

			find.y = i;


		}

	}
	//*/

	if (result.size() != 0) Recursive_For_Only_Edge_Tracking_Graph(copy, find.x, find.y, result);

	//Recursive_For_Only_Edge_Tracking(copy, wrist.x, wrist.y, result);

	//if (result.size() != 0) imshow("Check", copy);

	return result;
}

Mat Check_Tracking_Graph(Mat origin, vector<Point> points)
{
	Mat result = Mat::zeros(origin.size(), CV_8UC3);

	if (points.size() != 0)
	{
		for (int i = 0; i < points.size() - 1; i++)
		{
			line(result, points[i], points[i + 1], Scalar(0, 255, 0), 1);
			//result.at<Vec3b>(points[i].y, points[i].x)[0] = 0;
			//result.at<Vec3b>(points[i].y, points[i].x)[1] = 1;
			//result.at<Vec3b>(points[i].y, points[i].x)[2] = 0;
		}
	}

	return result;
}

vector<save_Point> Edge_Tracking_Simple(Mat origin)
{
	Mat copy = origin.clone();

	vector<save_Point> result;

	save_Point temp, find;

	///*
	for (int i = 0; i < copy.size().height; i++)
	{
		for (int j = 0; j < copy.size().width; j++)
		{
			if (copy.at<uchar>(i, j) == 255)
			{
				temp.x_Point = j;
				temp.y_Point = i;

				result.push_back(temp);
			}
		}
	}
	//*/

	return result;
}

Mat Check_Tracking(Mat origin, vector<save_Point> points)
{
	Mat result = Mat::zeros(origin.size(), CV_8UC3);

	if (points.size() != 0)
	{
		for (int i = 0; i < points.size(); i++)
		{
			result.at<Vec3b>(points[i].y_Point, points[i].x_Point)[0] = 0;
			result.at<Vec3b>(points[i].y_Point, points[i].x_Point)[1] = 0;
			result.at<Vec3b>(points[i].y_Point, points[i].x_Point)[2] = 255;
		}
	}

	return result;
}

double Distance_of_Two_Dots(Point a, Point b)
{
	return sqrt(pow((double)(a.x - b.x), 2.0) + pow((double)(a.y - b.y), 2.0));
}

vector<Point> Normalize_Degree(vector<Point2d> degree)
{

	int j, count, current = 0;

	double y_sum;

	Point temp;

	vector<Point> result;

	if (degree.size() != 0)
	{
		for (int i = 0; i < 360; i++)
		{
			y_sum = 0.0;

			count = 0;

			for (j = current; degree[j].x <= i; j++)
			{
				y_sum += degree[j].y;

				count++;
			}

			current = j;

			temp.x = i;

			if (count != 0) temp.y = y_sum / count;
			else temp.y = 0;

			result.push_back(temp);

		}
	}

	return result;
}

double Find_Degree(Point2d a, Point2d b)
{
	double temp, theta, degree;

	Point2d vec1, vec2;

	vec1 = a;

	vec2 = b;

	temp = sqrt(a.x*a.x + a.y*a.y);

	vec1.x /= temp;
	vec1.y /= temp;

	temp = sqrt(b.x*b.x + b.y*b.y);

	vec2.x /= temp;
	vec2.y /= temp;

	theta = vec1.x*vec2.x + vec1.y*vec2.y;

	theta = acos(theta);

	degree = theta * (180 / 3.142592);

	return degree;

}

double Find_Degree2(Point2d a, Point2d b)
{
	double temp1, temp2, theta, degree;

	Point2d vec1, vec2;

	vec1 = a;

	vec2 = b;

	temp1 = sqrt(a.x*a.x + a.y*a.y);

	temp2 = sqrt(b.x*b.x + b.y*b.y);

	theta = (vec1.x*vec2.y - vec1.y*vec2.x) / (temp1 * temp2);

	theta = asin(theta);

	degree = theta * (180 / 3.142592);

	return degree;

}

void Check_Value_vector(vector<Point> vec, int count)
{
	FILE* pFile;
	string str;

	str = format("check_degree_%d.txt", count);

	pFile = fopen(str.c_str(), "w");

	if (vec.size() != 0)
	{
		for (int i = 0; i < vec.size(); i++)
		{
			str = format("%d	%d", vec[i].x, vec[i].y);
			fputs(str.c_str(), pFile);
			fputs("\n", pFile);
		}
	}

	fclose(pFile);
}

void Check_Value_vector_Full(vector<vector<Point>> vec, int count)
{
	FILE* pFile;
	string str;

	str = format("check_degree_Full_%d.txt", count);

	pFile = fopen(str.c_str(), "w");

	if (vec.size() != 0)
	{
		for (int i = 0; i < vec.size(); i++)
		{
			for (int j = 0; j < vec[i].size(); j++)
			{
				str = format("%d	%d", vec[i][j].x, vec[i][j].y);
				fputs(str.c_str(), pFile);
				fputs("\n", pFile);
			}
		}
	}

	fclose(pFile);
}

void Check_Value_vector_Full_Smooth(vector<vector<Point>> vec, int count)
{
	FILE* pFile;
	string str;

	str = format("check_degree_Full_Smooth_%d.txt", count);

	pFile = fopen(str.c_str(), "w");

	if (vec.size() != 0)
	{
		for (int i = 0; i < vec.size(); i++)
		{
			for (int j = 0; j < vec[i].size(); j++)
			{
				str = format("%d	%d", vec[i][j].x, vec[i][j].y);
				fputs(str.c_str(), pFile);
				fputs("\n", pFile);
			}
		}
	}

	fclose(pFile);
}

void Check_Value_Normalize(vector<Point> vec, int count)
{
	FILE* pFile;
	string str;

	str = format("check_degree_normal_%d.txt", count);

	pFile = fopen(str.c_str(), "w");

	if (vec.size() != 0)
	{
		for (int i = 0; i < vec.size(); i++)
		{
			str = format("%d	%d", vec[i].x, vec[i].y);
			fputs(str.c_str(), pFile);
			fputs("\n", pFile);
		}
	}

	fclose(pFile);
}

vector<Point> Turn_Degree_int(vector<Point2d> degree, int max_height)
{
	vector<Point> result;

	Point temp;

	if (degree.size() != 0)
	{
		for (int i = 0; i < degree.size(); i++)
		{
			temp.x = degree[i].x;
			temp.y = degree[i].y;
			if (temp.y >= (max_height + 10)) temp.y = (max_height + 10) - 1;
			else if (temp.y <= 0) temp.y = 0;
			if (temp.x >= 360) temp.x = 360 - 1;
			else if (temp.x <= 0) temp.x = 0;

			result.push_back(temp);
		}
	}

	return result;
}

Mat Show_Degree_Graph(vector<Point> degree, int max_height)
{
	Point temp;

	Mat result;

	if (degree.size() != 0)
	{
		result = Mat::zeros(max_height + 10, 360, CV_8UC3);

		for (int i = 0; i < degree.size(); i++)
		{
			temp.x = degree[i].x;
			temp.y = result.size().height - degree[i].y;

			result.at<Vec3b>(temp.y, temp.x)[2] = 255;
		}
	}

	return result;
}

Mat Show_Degree_Graph_Full(vector<vector <Point>> degree, int max_height)
{
	Point temp;

	Mat result;

	if (degree.size() != 0)
	{
		result = Mat::zeros(max_height + 10, 360, CV_8UC3);

		for (int i = 0; i < degree.size(); i++)
		{
			for (int j = 0; j < degree[i].size(); j++)
			{
				temp.x = degree[i][j].x;
				temp.y = (result.size().height - 1) - degree[i][j].y;

				result.at<Vec3b>(temp.y, temp.x)[2] = 255;
			}
		}
	}

	return result;
}

vector<vector <Point>> Thresholding(vector<vector <Point>> degree, int thres)
{
	vector<vector <Point>> result;

	vector<Point> temp_Point;

	Point temp;

	if (degree.size() != 0)
	{
		for (int i = 0; i < degree.size(); i++)
		{
			temp_Point.clear();

			for (int j = 0; j < degree[i].size(); j++)
			{
				if (degree[i][j].y > thres)
				{
					temp.x = degree[i][j].x;
					temp.y = degree[i][j].y - thres;

					temp_Point.push_back(temp);
				}
			}

			if (temp_Point.size() != 0) result.push_back(temp_Point);
		}
	}

	return result;

}

vector<vector <Point>> Make_Full_Sort(vector<Point> degree)
{
	vector<vector <Point>> result;

	vector<Point> temp_Point;

	Point temp;

	if (degree.size() != 0)
	{
		QuickSort_Point(degree, 0, degree.size() - 1);

		for (int i = 0; i < degree.size(); i++)
		{
			if (i != 0)
			{
				if (temp_Point[0].x != degree[i].x)
				{
					QuickSort_PointY(temp_Point, 0, temp_Point.size() - 1);

					result.push_back(temp_Point);

					temp_Point.clear();
				}
			}

			temp_Point.push_back(degree[i]);
		}
	}


	return result;
}

vector<vector <Point>> Make_Smooth_Full(vector<vector <Point>> origin, int range)
{
	int sum, count;

	Point temp;

	vector<Point> temp_Point;

	vector<vector <Point>> result;

	if (origin.size() != 0)
	{
		for (int i = 0; i < origin.size(); i++)
		{
			sum = 0;

			count = 0;

			temp.x = 0;

			temp.y = 0;

			temp_Point.clear();

			for (int j = 0; j < origin[i].size(); j++)
			{
				if (j == 0)
				{
					temp = origin[i][j];

					sum = temp.y;

					count = 1;
				}
				if ((j != 0) && (abs(temp.y - origin[i][j].y) <= range))
				{
					sum += origin[i][j].y;

					count += 1;
				}
				else if ((j != 0) && (abs(temp.y - origin[i][j].y) > range))
				{
					temp.y = sum / count;

					temp_Point.push_back(temp);

					temp = origin[i][j];

					sum = temp.y;

					count = 1;
				}
			}

			temp.y = sum / count;

			temp_Point.push_back(temp);

			result.push_back(temp_Point);
		}
	}

	return result;
}

void Find_Thres_Point(vector<vector <Point>> origin, vector<Point> &thres_Data, int thres)
{
	int flag = 0, range = 10;

	Point min_A, min_B;

	vector<Point> temp;

	double slope, constant, temp_Double, temp_Dist, min_Dist;

	if (origin.size() != 0)
	{
		for (int i = 0; i < origin.size() - 1; i++)
		{
			for (int j = 0; j < origin[i].size(); j++)
			{

				if (abs(thres - origin[i][j].y) <= range)
				{

					min_Dist = 0.0;

					min_A.x = 0;
					min_A.y = 0;

					min_B.x = 0;
					min_B.y = 0;

					for (int k = 0; k < origin[i + 1].size(); k++)
					{

						temp_Dist = Distance_of_Two_Dots(origin[i][j], origin[i + 1][k]);

						if (min_Dist == 0.0) min_Dist = temp_Dist;

						if (temp_Dist <= min_Dist)
						{
							min_A = origin[i][j];

							min_B = origin[i + 1][k];
						}
					}

					slope = ((double)min_B.y - (double)min_A.y) / ((double)min_B.x - (double)min_A.x);

					constant = (double)min_B.y - slope * (double)min_B.x;

					temp_Double = ((double)thres - constant) / slope;

					if (((int)temp_Double >= min_A.x) && ((int)temp_Double <= min_B.x) && (min_Dist <= 5.0))
					{
						if (min_A.y >= thres)	thres_Data.push_back(min_A);
						else if (min_B.y >= thres)	thres_Data.push_back(min_B);

						//thres_Data.push_back(min_A);

						//thres_Data.push_back(min_B);
					}

				}

			}
		}
	}
}

Mat Check_Thres_Point(Mat origin, vector<Point> thres_Data)
{
	Mat result = origin.clone();

	if (thres_Data.size() != 0)
	{
		for (int j = 0; j < thres_Data.size(); j++)
		{
			result.at<Vec3b>(result.size().height - thres_Data[j].y, thres_Data[j].x)[1] = 255;
			result.at<Vec3b>(result.size().height - thres_Data[j].y, thres_Data[j].x)[2] = 0;
		}
	}

	return result;
}

Mat Fill_Green(Mat origin, vector<vector <Point>> dots)
{
	Mat result = origin.clone();

	if ((origin.size().width != 0) && (dots.size() != 0))
	{
		for (int i = 0; i < dots.size(); i++)
		{
			for (int j = 0; j < dots[i].size(); j++)
			{
				if (dots[i].size() % 2 == 1)
				{
					if (j % 2 == 0)
					{
						if (j == 0)
						{
							for (int k = 0; k < dots[i][j].y; k++)
							{
								result.at<Vec3b>((result.size().height - 1) - k, dots[i][j].x)[1] = 255;
								result.at<Vec3b>((result.size().height - 1) - k, dots[i][j].x)[2] = 0;
							}
						}
						else
						{
							for (int k = dots[i][j - 1].y + 1; k < dots[i][j].y; k++)
							{
								result.at<Vec3b>((result.size().height - 1) - k, dots[i][j].x)[1] = 255;
								result.at<Vec3b>((result.size().height - 1) - k, dots[i][j].x)[2] = 0;
							}
						}
					}
				}
				/*
				else if (dots[i].size() % 2 == 0)
				{
				if (j % 2 == 1)
				{
				for (int k = dots[i][j - 1].y + 1; k < dots[i][j].y; k++)
				{
				result.at<Vec3b>((result.size().height - 1) - k, dots[i][j].x)[1] = 255;
				result.at<Vec3b>((result.size().height - 1) - k, dots[i][j].x)[2] = 0;
				}
				}
				}
				*/
			}
		}
	}

	return result;
}

Mat Degree_Tracking(vector<vector <Point>> degree, int max_Height)
{
	Mat result = Mat::zeros(max_Height + 10, 360, CV_8UC3);;

	vector<vector <Point>> copy(degree);

	double temp_Dist, min_Dist;

	int range, flag = 0;

	Point start_Locate, start_Point, temp_Locate, temp_Point;

	vector<Point> track;

	if (degree.size() != 0)
	{
		range = 3;

		start_Locate.x = 0;
		start_Locate.y = 0;

		start_Point = copy[start_Locate.x][start_Locate.y];

		track.push_back(start_Point);

		copy[start_Locate.x].erase(copy[start_Locate.x].begin() + start_Locate.y);

		//copy[start_Locate.x][start_Locate.y].x = 0;
		//copy[start_Locate.x][start_Locate.y].y = 0;

		while (flag != 1)
		{
			temp_Locate.x = 0;
			temp_Locate.y = 0;

			temp_Point.x = 0;
			temp_Point.y = 0;

			min_Dist = 0.0;

			temp_Dist = 0.0;

			for (int i = start_Locate.x - range; i <= start_Locate.x + range; i++)
			{
				///*
				if ((i >= 0) && (i < copy.size()))
				{
					if (copy[i].size() != 0)
					{

						for (int j = 0; j < copy[i].size(); j++)
						{
							//if ((copy[i][j].x != 0) && (copy[i][j].y != 0))
							{
								temp_Dist = Distance_of_Two_Dots(start_Point, copy[i][j]);

								if (min_Dist == 0.0)
									min_Dist = temp_Dist;

								if ((temp_Dist <= min_Dist) && (temp_Dist >= 1.0))
								{

									min_Dist = temp_Dist;

									temp_Locate.x = i;
									temp_Locate.y = j;

									temp_Point = copy[i][j];

								}
							}

						}

					}
				}
				//*/
			}
			if ((temp_Locate.x == 0) && (temp_Locate.y == 0)) flag = 1;

			else{
				line(result, start_Point, temp_Point, Scalar(0, 255, 0), 1);

				start_Locate = temp_Locate;

				start_Point = temp_Point;

				track.push_back(start_Point);

				copy[start_Locate.x].erase(copy[start_Locate.x].begin() + start_Locate.y);
				//copy[start_Locate.x][start_Locate.y].x = 0;
				//copy[start_Locate.x][start_Locate.y].y = 0;
			}
		}

	}

	//line(result, Point(100, 100), Point(20, 20), Scalar(0, 255, 0), 1);

	return result;
}

Mat Each_Region_Regresssion(vector<Point> degree, int max_Height, int region_Size, int polynomal)
{
	vector<Point> temp_Point;

	vector<Point> result;

	Mat coefficient;

	Mat show_Result = Mat::zeros(max_Height + 10, 360, CV_8UC3);

	double y;

	int count;

	if (degree.size() != 0)
	{
		count = 0;

		for (int i = 0; i < show_Result.size().width; i++)
		{
			temp_Point.clear();

			for (int j = count; ((j < degree.size()) && (degree[j].x < (i + region_Size))); j++)
			{
				temp_Point.push_back(degree[j]);

				count++;
			}

			if (temp_Point.size() != 0)
			{

				coefficient = Linear_Regression(temp_Point, polynomal);

				for (int l = 0; l < temp_Point.size(); l++)
				{
					y = 0.0;

					for (int k = 0; k < coefficient.size().height; k++)
					{
						y += coefficient.at<double>(k, 0) * pow((double)temp_Point[l].x, (double)k);
					}

					if (y >= show_Result.size().height) y = show_Result.size().height - 1;
					else if ((int)y <= 0) y = 0.0;


					//result.push_back(degree[j]);


					show_Result.at<Vec3b>((int)y, temp_Point[l].x)[0] = 0;
					show_Result.at<Vec3b>((int)y, temp_Point[l].x)[1] = 255;
					show_Result.at<Vec3b>((int)y, temp_Point[l].x)[2] = 0;

				}
			}

			i += region_Size;

		}
	}

	return show_Result;
}

Mat Convert_All_Pixel_Gray(Mat origin)
{
	Mat result = Mat::zeros(origin.size(), CV_8U);

	for (int i = 0; i < origin.size().height; i++)
	{
		for (int j = 0; j < origin.size().width; j++)
		{
			if (origin.at<Vec3b>(i, j)[2] != 0)
			{
				//result.at<Vec3b>(i, j)[0] = 0;
				//result.at<Vec3b>(i, j)[1] = 0;
				//result.at<Vec3b>(i, j)[2] = 255;

				result.at<uchar>(i, j) = 255;
			}
		}
	}

	return result;
}

Mat Distict_Thres(Mat origin, int thres, vector<FEMD>& recent, string name)
{
	Mat result = Mat::zeros(origin.size().height - thres, origin.size().width, CV_8UC3);

	vector<FEMD> femd_Data;

	vector<Point> temp_Point;

	FEMD femd_Temp;

	int for_Color = 0.0;

	char str[200];

	string backname, fullname;

	Mat debug = Mat::zeros(360, 360, CV_8UC3);

	for (int i = 0; i < origin.size().height - thres; i++)
	{
		for (int j = 0; j < origin.size().width; j++)
		{
			result.at<Vec3b>(i, j) = origin.at<Vec3b>(i, j);
		}
	}

	for (int k = 0; k < result.size().width - 2; k++)
	{
		femd_Temp.start = Point(0, 0);
		femd_Temp.end = Point(0, 0);
		femd_Temp.Size = 0;

		if ((result.at<Vec3b>(result.size().height - 1, k)[0] == 255) && (result.at<Vec3b>(result.size().height - 1, k)[1] == 255) && (result.at<Vec3b>(result.size().height - 1, k)[2] == 255))
		{
			if (((result.at<Vec3b>(result.size().height - 1, k + 1)[0] == 0) && (result.at<Vec3b>(result.size().height - 1, k + 1)[1] == 255) && (result.at<Vec3b>(result.size().height - 1, k + 1)[2] == 0)) ||
				((result.at<Vec3b>(result.size().height - 1, k + 2)[0] == 0) && (result.at<Vec3b>(result.size().height - 1, k + 2)[1] == 255) && (result.at<Vec3b>(result.size().height - 1, k + 2)[2] == 0)))
			{
				femd_Temp.start = Point(k, result.size().height);

				for (int l = k + 2; l < result.size().width; l++)
				{
					if ((result.at<Vec3b>(result.size().height - 1, l)[0] == 255) && (result.at<Vec3b>(result.size().height - 1, l)[1] == 255) && (result.at<Vec3b>(result.size().height - 1, l)[2] == 255))
					{
						femd_Temp.end = Point(l, result.size().height);

						break;
					}
				}

				if (femd_Temp.end != Point(0, 0))
				{
					//if (Distance_of_Two_Dots(femd_Temp.start, femd_Temp.end) <= 50.0)
					{
						floodFill(result, Point(k + 2, result.size().height - 1), Scalar(255.0 - for_Color, 0.0 + for_Color, 0.0 + for_Color * 2));

						for (int m = 0; m < result.size().height; m++)
						{
							for (int n = 0; n < result.size().width; n++)
							{
								if ((result.at<Vec3b>(m, n)[0] == 255.0 - for_Color) && (result.at<Vec3b>(m, n)[1] == 0.0 + for_Color) && (result.at<Vec3b>(m, n)[2] == 0.0 + for_Color * 2))
									femd_Temp.Size += 1;
							}
						}

						for_Color += 25.0;

						femd_Data.push_back(femd_Temp);

					}
				}
			}

		}
	}

	for (int o = 0; o < femd_Data.size(); o++)
	{

		sprintf(str, "p1 : %d, %d / p2 : %d, %d / size : %d", femd_Data[o].start.x, femd_Data[o].start.y, femd_Data[o].end.x, femd_Data[o].end.y, femd_Data[o].Size);

		putText(debug, str, Point(10, 15 + 15 * o), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));

		recent.push_back(femd_Data[o]);

	}

	backname = format(" debug");

	fullname = name + backname;

	imshow(fullname.c_str(), debug);

	return result;
}

Mat Set_FEMD_Base(vector<FEMD> recent, vector<FEMD>& base)
{
	Mat base_Show = Mat::zeros(360, 360, CV_8UC3);

	char str[200];

	if (recent.size() != 0)
	{
		base.clear();
		for (int i = 0; i < recent.size(); i++)
		{
			base.push_back(recent[i]);
		}
	}

	for (int o = 0; o < base.size(); o++)
	{

		sprintf(str, "p1 : %d, %d / p2 : %d, %d / size : %d", base[o].start.x, base[o].start.y, base[o].end.x, base[o].end.y, base[o].Size);

		putText(base_Show, str, Point(10, 15 + 15 * o), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));

	}

	//imshow("base_Show", base_Show);

	return base_Show;

}

Mat Compare_FEMD(vector<FEMD> base, vector<FEMD> recent, vector<Point>& finger_Result)
{

	Mat result_Show = Mat::zeros(360, 360, CV_8UC3);

	char str[200];

	int dist, dist_Thres = 15, size_Thres = 300, for_Line = 0, size_Sum;

	Point temp;

	if ((base.size() != 0) && (recent.size() != 0))
	{
		for (int i = 0; i < base.size(); i++)
		{
			for (int j = 0; j < recent.size(); j++)
			{
				dist = min(abs(base[i].start.x - recent[j].start.x), abs(base[i].end.x - recent[j].end.x));

				if (dist <= dist_Thres)
				{
					if (abs(base[i].Size - recent[j].Size) <= size_Thres)
					{
						sprintf(str, "%d class", i + 1);

						temp.x = i + 1;

						temp.y = 0;

						finger_Result.push_back(temp);

						putText(result_Show, str, Point(10, 15 + 15 * for_Line), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));

						for_Line++;
					}
					else
					{
						size_Sum = 0;

						for (int k = i; k < base.size(); k++)
						{
							dist = min(abs(base[k].start.x - recent[j].start.x), abs(base[k].end.x - recent[j].end.x));

							size_Sum += base[k].Size;

							if (dist <= dist_Thres)
							{
								if (abs(size_Sum - recent[j].Size) <= size_Thres)
								{
									sprintf(str, "%d ~ %d class", i + 1, k + 1);

									temp.x = i + 1;

									temp.y = k + 1;

									finger_Result.push_back(temp);

									putText(result_Show, str, Point(10, 15 + 15 * for_Line), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));

									for_Line++;
								}
							}
						}
					}
				}
			}
		}

		//imshow("result_Show", result_Show);
	}


	return result_Show;

}


vector<Point> Edge_Rotation(vector<save_Point> vec, Point wrist, save_Point center, int count, int max_Height, int thres, string name, vector<FEMD> &recent)
{
	vector<Point> degree;

	vector<vector <Point>> degree_Full, degree_Full_Smooth;

	vector<Point> after_Thres, thres_Up, thres_Down, thres_Data;

	Point temp;

	Mat temp_Gray;

	Mat coefficient;

	Mat check_Coefficient = Mat::zeros(max_Height + 10, 360, CV_8UC3);

	Mat check, check_Gaussian, check_Thinning;

	double slope, constant, max_Value = 0.0;

	string backname, fullname;

	if (vec.size() != 0)
	{

		slope = (((double)wrist.y * 2) - (double)center.y_Point) / (((double)wrist.x * 2) - (double)center.x_Point);

		constant = ((double)wrist.y * 2) - slope * ((double)wrist.x * 2);

		for (int i = 0; i < vec.size(); i++)
		{

			temp.x = (int)Find_Degree(Point2d((double)wrist.x * 2 - (double)center.x_Point, (double)wrist.y * 2 - (double)center.y_Point), Point2d((double)vec[i].x_Point - (double)center.x_Point, (double)vec[i].y_Point - (double)center.y_Point));

			temp.y = Distance_of_Two_Dots(Point(vec[i].x_Point, vec[i].y_Point), Point(center.x_Point, center.y_Point));

			//if ((vec[i].y_Point >= (slope * vec[i].x_Point + constant)) && (slope <= 0)) temp.x = (180.0 - temp.x) + 180;
			//else if ((vec[i].y_Point < (slope * vec[i].x_Point + constant)) && (slope > 0)) temp.x = (180.0 - temp.x) + 180;

			if (Find_Degree2(Point2d((double)wrist.x * 2 - (double)center.x_Point, (double)wrist.y * 2 - (double)center.y_Point), Point2d((double)vec[i].x_Point - (double)center.x_Point, (double)vec[i].y_Point - (double)center.y_Point)) < 0)
				temp.x = (180.0 - temp.x) + 180;


			if (temp.y <= 0) temp.y = 0;
			else if (temp.y >= (int)max_Height) temp.y = (int)max_Height;
			if (temp.x <= 0) temp.x = 0;
			else if (temp.x >= 360) temp.x = 359;

			degree.push_back(temp);

			if (max_Value < temp.y) max_Value = temp.y;

		}

		//Normalized y value
		for (int j = 0; j < degree.size(); j++)
		{
			degree[j].y = (int)(degree[j].y * ((double)max_Height / max_Value));

			if (degree[j].y >= (int)max_Height) degree[j].y = (int)max_Height;
		}

		//QuickSort_Point(degree, 0, degree.size() - 1);

		//Check_Value_vector(degree, count);


		check = Show_Degree_Graph(degree, max_Height);

		//medianBlur(check, check_Median, 3);
		//blur(check, check_Median, Size(5, 5));
		GaussianBlur(check, check_Gaussian, Size(3, 3), 0, 0);

		check_Thinning = Convert_All_Pixel_Gray(check_Gaussian);

		backname = format(" Check_Median");

		fullname = name + backname;

		imshow(fullname.c_str(), check_Thinning);

		thinning(check_Thinning);

		backname = format(" Check_Thinning");

		fullname = name + backname;

		imshow(fullname.c_str(), check_Thinning);

		cvtColor(check_Thinning, check_Thinning, CV_GRAY2BGR);

		floodFill(check_Thinning, Point(0, check_Thinning.size().height - 1), Scalar(0.0, 255.0, 0.0));

		backname = format(" Fill_Green");

		fullname = name + backname;

		imshow(fullname.c_str(), check_Thinning);

		backname = format(" Thres");

		fullname = name + backname;

		imshow(fullname.c_str(), Distict_Thres(check_Thinning, thres, recent, name));

	}

	return degree;
}

/////////////////////////////////////// For RGB to HSV Convert /////////////////////////////////////////////

#define MATH_MIN3(x,y,z)		( (y) <= (z) ? ((x) <= (y) ? (x) : (y)) : ((x) <= (z) ? (x) : (z)) )
#define MATH_MAX3(x,y,z)		( (y) >= (z) ? ((x) >= (y) ? (x) : (y)) : ((x) >= (z) ? (x) : (z)) )

struct hsv_color {
	unsigned char h;        // Hue: 0 ~ 255 (red:0, gree: 85, blue: 171)
	unsigned char s;        // Saturation: 0 ~ 255
	unsigned char v;        // Value: 0 ~ 255
};
/*
hsv_color RGB2HSV(unsigned char r, unsigned char g, unsigned char b)
{
unsigned char rgb_min, rgb_max;
rgb_min = MATH_MIN3(b, g, r);
rgb_max = MATH_MAX3(b, g, r);

hsv_color hsv;
hsv.v = rgb_max;
if (hsv.v == 0) {
hsv.h = hsv.s = 0;
return hsv;
}

hsv.s = 255 * (rgb_max - rgb_min) / hsv.v;
if (hsv.s == 0) {
hsv.h = 0;
return hsv;
}

if (rgb_max == r) {
hsv.h = 0 + 43 * (g - b) / (rgb_max - rgb_min);
}
else if (rgb_max == g) {
hsv.h = 85 + 43 * (b - r) / (rgb_max - rgb_min);
}
else  {
hsv.h = 171 + 43 * (r - g) / (rgb_max - rgb_min);
}

return hsv;
}
*/
hsv_color RGB2HSV(unsigned char r, unsigned char g, unsigned char b)
{
	hsv_color         out;

	float fH, fS, fV;

	float fCMax = max(max((float)r, (float)g), (float)b);
	float fCMin = min(min((float)r, (float)g), (float)b);
	float fDelta = fCMax - fCMin;
	if (fDelta > 0) {
		if (fCMax == (float)r) {
			fH = 60 * (fmod((((float)g - (float)b) / fDelta), 6));
		}
		else if (fCMax == (float)g) {
			fH = 60 * ((((float)b - (float)r) / fDelta) + 2);
		}
		else if (fCMax == (float)b) {
			fH = 60 * ((((float)r - (float)g) / fDelta) + 4);
		}
		if (fCMax > 0) {
			fS = 255 * fDelta / fCMax;
		}
		else {
			fS = 0;
		}
		fV = fCMax;
	}
	else {
		fH = 0;
		fS = 0;
		fV = fCMax;
	}
	if (fH < 0) {
		fH = 360 + fH;
	}

	out.h = fH;
	out.s = fS;
	out.v = fV;

	return out;
}

struct YCbCr{

	unsigned char Y;
	unsigned char Cb;
	unsigned char Cr;

};

YCbCr RGB_to_YCbCr(unsigned char r, unsigned char g, unsigned char b)
{
	YCbCr newPixel;

	newPixel.Y = 0.299 * r + 0.587 * g + 0.114 * b;
	newPixel.Cb = -0.168736 * r - 0.331264 * g + 0.5 * b + 128.0;
	newPixel.Cr = 0.5 * r - 0.418688 * g - 0.081312 * b + 128.0;

	return newPixel;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////

Point Center_of_Two_Dots(Point a, Point b)
{
	return Point(((a.x + b.x) / 2), ((a.y + b.y) / 2));
}

vector<Point> Sum_Vector_Points(vector<Point> a, vector<Point> b)
{
	vector<Point> sum;

	sum.clear();
	sum.assign(a.begin(), a.end());

	for (int i = 0; i < b.size(); i++)
	{
		sum.push_back(b[i]);
	}

	return sum;

}

vector<square_Data> Find_Distinct_Data(Mat origin, int Square_Size, int thres_Data, double distance_Group)	//Input Mat is binary set
{
	Mat copy = origin.clone();

	vector<square_Data> square_Group;

	square_Data square_Temp;

	int count;

	int max_X, max_Y;

	Point temp_Max;

#pragma omp parallel for
	for (int i = 0; i < copy.size().height; i++)
	{
		for (int j = 0; j < copy.size().width; j++)
		{
			if (copy.at<uchar>(i, j) >= 255)
			{
				count = 0;

				square_Temp.left_Up = Point(j, i);

				max_X = square_Temp.left_Up.x;

				max_Y = square_Temp.left_Up.y;

				for (int k = i; (k < (i + Square_Size)); k++)
				{
					for (int l = j; (l < (j + Square_Size)); l++)
					{
						if ((k < copy.size().height) && (l < copy.size().width))
						{
							if (copy.at<uchar>(k, l) >= 255)
							{
								copy.at<uchar>(k, l) = 0;

								//cout << "done" << endl;

								square_Temp.Data.push_back(Point(l, k));

								if (l > max_X) max_X = l;
								if (k > max_Y) max_Y = k;

								count++;
							}
						}
					}
				}

				if (count >= thres_Data)
				{
					//square_Temp.left_Up = Point(j, i);

					/*
					if ((j + Square_Size) >= (copy.size().width - 1)) square_Temp.right_Down.x = copy.size().width - 1;
					else square_Temp.right_Down.x = j + Square_Size;
					if ((i + Square_Size) >= (copy.size().height - 1)) square_Temp.right_Down.y = copy.size().height - 1;
					else square_Temp.right_Down.y = i + Square_Size;
					*/
					square_Temp.right_Down.x = max_X;

					square_Temp.right_Down.y = max_Y;

					square_Group.push_back(square_Temp);

					square_Temp.Data.clear();

					//cout << "i : " << i << "	j : " << j << "	square_Temp.right_Down.x : " << square_Temp.right_Down.x << "	square_Temp.right_Down.y : " << square_Temp.right_Down.y << endl;

					//rectangle(copy, Point(j, i), Point(square_Temp.right_Down.x, square_Temp.right_Down.y), Scalar(255, 255, 255));


				}
				else square_Temp.Data.clear();

			}
		}
	}


	///*	
	//cout << square_Group.size() << endl;

	vector<square_Data> square_class;

	square_Data class_Origin;

	square_Data class_Temp;

	double center_Distance;


#pragma omp parallel for
	for (int m = 0; m < square_Group.size(); m++)
	{
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/*
		rectangle(copy, Point(square_Group[m].left_Up.x, square_Group[m].left_Up.y), Point(square_Group[m].right_Down.x, square_Group[m].right_Down.y), Scalar(255, 255, 255));

		for (int n = 0; n < square_Group[m].Data.size(); n++)
		{
		copy.at<uchar>(square_Group[m].Data[n].y, square_Group[m].Data[n].x) = 255;
		}
		*/
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// See All Rectangle of Groups

		class_Origin.left_Up = square_Group[m].left_Up;

		class_Origin.right_Down = square_Group[m].right_Down;

		class_Origin.Data.clear();

		class_Origin.Data.assign(square_Group[m].Data.begin(), square_Group[m].Data.end());


		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		for (int p = 0; p < square_Group.size(); p++)
		{

			if (m != p)
			{

				center_Distance = Distance_of_Two_Dots(Center_of_Two_Dots(class_Origin.left_Up, class_Origin.right_Down), Center_of_Two_Dots(square_Group[p].left_Up, square_Group[p].right_Down));

				if (center_Distance <= distance_Group)
				{
					if (class_Origin.left_Up.x < square_Group[p].left_Up.x) class_Temp.left_Up.x = class_Origin.left_Up.x;
					else class_Temp.left_Up.x = square_Group[p].left_Up.x;

					if (class_Origin.left_Up.y < square_Group[p].left_Up.y) class_Temp.left_Up.y = class_Origin.left_Up.y;
					else class_Temp.left_Up.y = square_Group[p].left_Up.y;

					if (class_Origin.right_Down.x < square_Group[p].right_Down.x) class_Temp.right_Down.x = square_Group[p].right_Down.x;
					else class_Temp.right_Down.x = class_Origin.right_Down.x;

					if (class_Origin.right_Down.y < square_Group[p].right_Down.y) class_Temp.right_Down.y = square_Group[p].right_Down.y;
					else class_Temp.right_Down.y = class_Origin.right_Down.y;

					class_Temp.Data = Sum_Vector_Points(class_Origin.Data, square_Group[p].Data);

					class_Origin.left_Up = class_Temp.left_Up;

					class_Origin.right_Down = class_Temp.right_Down;

					class_Origin.Data.clear();

					class_Origin.Data.assign(class_Temp.Data.begin(), class_Temp.Data.end());

					square_Group.erase(square_Group.begin() + p);

					p = 0;

				}
			}
		}

		square_Group[m].left_Up = class_Origin.left_Up;

		square_Group[m].right_Down = class_Origin.right_Down;

		square_Group[m].Data.clear();

		square_Group[m].Data.assign(class_Origin.Data.begin(), class_Origin.Data.end());

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Merge Gruops if they are nearly close & Divide Classes

	}


	return square_Group;

}

Mat Draw_Classes(vector<square_Data> square_Group, Size max_Size)
{
	Mat copy = Mat::zeros(max_Size, CV_8U);

	if (square_Group.size() != 0)
	{
		//cout << square_Group.size() << endl;

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/*
		for (int m = 0; m < square_Group.size(); m++)
		{
		rectangle(copy, Point(square_Group[m].left_Up.x, square_Group[m].left_Up.y), Point(square_Group[m].right_Down.x, square_Group[m].right_Down.y), Scalar(255, 255, 255));

		for (int n = 0; n < square_Group[m].Data.size(); n++)
		{
		copy.at<uchar>(square_Group[m].Data[n].y, square_Group[m].Data[n].x) = 255;
		}

		}
		*/
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// View All Class Rectangle

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///*
		int temp, maximum = 0, maximum_Num = 0;

		for (int m = 0; m < square_Group.size(); m++)
		{
			temp = square_Group[m].Data.size();

			if (temp > maximum)
			{
				maximum = temp;

				maximum_Num = m;
			}
		}

		//maximum_Num = 4;

		rectangle(copy, square_Group[maximum_Num].left_Up, square_Group[maximum_Num].right_Down, 255, 1, 4);

		for (int n = 0; n < square_Group[maximum_Num].Data.size(); n++)
		{
			copy.at<uchar>(square_Group[maximum_Num].Data[n].y, square_Group[maximum_Num].Data[n].x) = 255;
		}
		//*/
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// View Only Largest Rectangle
	}

	return copy;
}

vector<Point> Find_Largest_Class(vector<square_Data> square_Group)
{
	vector<Point> result;

	if (square_Group.size() != 0)
	{

		int temp, maximum = 0, maximum_Num = 0;

		for (int m = 0; m < square_Group.size(); m++)
		{
			temp = square_Group[m].Data.size();

			if (temp > maximum)
			{
				maximum = temp;

				maximum_Num = m;
			}
		}

		for (int n = 0; n < square_Group[maximum_Num].Data.size(); n++)
		{
			result.push_back(square_Group[maximum_Num].Data[n]);
		}

	}

	return result;
}

Mat Merge_Two_Mat_Data(Mat alpha, Mat beta)
{
	Mat copy = alpha.clone();

#pragma omp parallel for
	for (int i = 0; i < copy.size().height; i++)
	{
		for (int j = 0; j < copy.size().width; j++)
		{
			if (beta.at<uchar>(i, j) != 0)
			{
				copy.at<uchar>(i, j) = beta.at<uchar>(i, j);
			}
		}
	}

	return copy;
}

Point Center_Point_For_Each_Classes(vector<square_Data> square_Group, Size max_Size)
{
	Mat copy = Mat::zeros(max_Size, CV_8U);

	Point center;

	if (square_Group.size() != 0)
	{
		//cout << square_Group.size() << endl;

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///*
		int temp, maximum = 0, maximum_Num = 0;

#pragma omp parallel for
		for (int m = 0; m < square_Group.size(); m++)
		{
			temp = square_Group[m].Data.size();

			if (temp > maximum)
			{
				maximum = temp;

				maximum_Num = m;
			}
		}

		//center = square_Group[maximum_Num].left_Up;

		center = Center_of_Two_Dots(square_Group[maximum_Num].left_Up, square_Group[maximum_Num].right_Down);

		//*/
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Only Largest Rectangle
	}

	return center;
}

vector<square_Data> Delete_Out_Of_Range(vector<square_Data> square_Group, Mat depth, float base_Depth, float hand_Depth_Range)
{
	Mat copy = Mat::zeros(depth.size(), CV_8U);

	vector<square_Data> result;

	Point center;

	//result.clear();

	//result.assign(square_Group.begin(), square_Group.end());

	///*

	if (square_Group.size() != 0)
	{

		//#pragma omp parallel for
		for (int m = 0; m < square_Group.size(); m++)
		{
			///*
			center = Center_of_Two_Dots(square_Group[m].left_Up, square_Group[m].right_Down);

			if (abs((base_Depth - depth.at<float>(center.y, center.x))) <= hand_Depth_Range)
			{
				result.push_back(square_Group[m]);

			}
			else
				cout << "Error" << endl;
			//*/
			//result.push_back(square_Group[m]);
		}

	}

	//*/

	return result;

	//return square_Group;
}

vector<square_Data> Find_Belt(Mat origin, Mat Depth_Origin)
{
	Mat belt = Mat::zeros(origin.size().height, origin.size().width, CV_8U);

	YCbCr ybr_Temp;

	vector<square_Data> result;

#pragma omp parallel for
	for (int i = 0; i < origin.size().height; i++)
	{
		for (int j = 0; j < origin.size().width; j++)
		{

			if (Depth_Origin.at<float>(i, j) != 0)
			{
				ybr_Temp = RGB_to_YCbCr(origin.at<Vec3b>(i, j)[2], origin.at<Vec3b>(i, j)[1], origin.at<Vec3b>(i, j)[0]);

				if (((ybr_Temp.Y <= 20) && (ybr_Temp.Cb >= 125) && (ybr_Temp.Cb < 130)) && ((ybr_Temp.Cr >= 125) && (ybr_Temp.Cr < 130)))	belt.at<uchar>(i, j) = 255;

			}

		}
	}

	result = Find_Distinct_Data(belt, 30, 10, 100.0);


	return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//											End Added																			//////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Cut_Hand_Color(Mat color_Data, Mat left_Hand_Color, Mat right_Hand_Color, Point L_Hand, Point R_Hand, int size)
{

	Point begin;

	Point end;

	begin.x = L_Hand.x - (size - 1) / 2;
	begin.y = L_Hand.y - (size - 1) / 2;

	if (begin.x < 0) begin.x = 0;
	if (begin.y < 0) begin.y = 0;

	end.x = L_Hand.x + (size - 1) / 2;
	end.y = L_Hand.y + (size - 1) / 2;

	if (end.x >= color_Data.size().width) end.x = color_Data.size().width;
	if (end.y >= color_Data.size().height) end.y = color_Data.size().height;

#pragma omp parallel for
	for (int i = begin.y; i < end.y; i++)
	{
		for (int j = begin.x; j < end.x; j++)
		{
			left_Hand_Color.at<Vec4b>(i - begin.y, j - begin.x) = color_Data.at<Vec4b>(i, j);
		}
	}

	begin.x = R_Hand.x - (size - 1) / 2;
	begin.y = R_Hand.y - (size - 1) / 2;

	if (begin.x < 0) begin.x = 0;
	if (begin.y < 0) begin.y = 0;

	end.x = R_Hand.x + (size - 1) / 2;
	end.y = R_Hand.y + (size - 1) / 2;

	if (end.x >= color_Data.size().width) end.x = color_Data.size().width;
	if (end.y >= color_Data.size().height) end.y = color_Data.size().height;

#pragma omp parallel for
	for (int i = begin.y; i < end.y; i++)
	{
		for (int j = begin.x; j < end.x; j++)
		{
			right_Hand_Color.at<Vec4b>(i - begin.y, j - begin.x) = color_Data.at<Vec4b>(i, j);
		}
	}
}

void CBodyBasics::Cut_Hand_Color_Float(Mat color_Data, Mat left_Hand_Color, Mat right_Hand_Color, Point L_Hand, Point R_Hand, D2D1_POINT_2F jointPoints_For_Hand[], int size, float range)
{

	Point begin;

	Point end;

	Point left_Point, right_Point;

	begin.x = L_Hand.x - (size - 1) / 2;
	begin.y = L_Hand.y - (size - 1) / 2;

	if (begin.x < 0) begin.x = 0;
	if (begin.y < 0) begin.y = 0;

	end.x = L_Hand.x + (size - 1) / 2;
	end.y = L_Hand.y + (size - 1) / 2;

	if (end.x >= color_Data.size().width) end.x = color_Data.size().width;
	if (end.y >= color_Data.size().height) end.y = color_Data.size().height;

#pragma omp parallel for
	for (int i = begin.y; i < end.y; i++)
	{
		for (int j = begin.x; j < end.x; j++)
		{
			if ((color_Data.at<float>(i, j) <= (color_Data.at<float>(L_Hand.y, L_Hand.x) + range)))
			{
				left_Hand_Color.at<float>(i - begin.y, j - begin.x) = color_Data.at<float>(i, j);
			}
			else left_Hand_Color.at<float>(i - begin.y, j - begin.x) = 0.0;

		}
	}

	begin.x = R_Hand.x - (size - 1) / 2;
	begin.y = R_Hand.y - (size - 1) / 2;

	if (begin.x < 0) begin.x = 0;
	if (begin.y < 0) begin.y = 0;

	end.x = R_Hand.x + (size - 1) / 2;
	end.y = R_Hand.y + (size - 1) / 2;

	if (end.x >= color_Data.size().width) end.x = color_Data.size().width;
	if (end.y >= color_Data.size().height) end.y = color_Data.size().height;

#pragma omp parallel for
	for (int i = begin.y; i < end.y; i++)
	{
		for (int j = begin.x; j < end.x; j++)
		{
			if ((color_Data.at<float>(i, j) <= (color_Data.at<float>(R_Hand.y, R_Hand.x) + range)))
			{
				right_Hand_Color.at<float>(i - begin.y, j - begin.x) = color_Data.at<float>(i, j);
			}
			else right_Hand_Color.at<float>(i - begin.y, j - begin.x) = 0.0;
		}
	}
}

void Xml_Write(Mat origin, string filename, string lable)
{
	FileStorage fs_C(filename, FileStorage::WRITE);
	fs_C << lable << origin;
	fs_C.release();
}

Mat Xml_Read(string filename, string lable)
{
	Mat result;

	FileStorage Read;
	Read.open(filename, FileStorage::READ);
	Read[lable] >> result;
	Read.release();

	return result;
}


static double angle(Point pt1, Point pt2, Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void setLabel(Mat& im, const string label, vector<Point>& contour)
{
	int fontface = FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;
	Size text = getTextSize(label, fontface, scale, thickness, &baseline);
	Rect r = boundingRect(contour);
	Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	rectangle(im, pt + Point(0, baseline), pt + Point(text.width, -text.height), CV_RGB(255, 255, 255), CV_FILLED);
	putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

Mat Crop_Regressiobn(Mat origin, Mat coefficient, Point& wrist)
{
	Mat output_Result = Mat::zeros(origin.size(), CV_8U);

	double y;

	Point temp_Point, s_Point;


	s_Point.x = 0.0;
	s_Point.y = 0.0;

	for (int i = 1; i < origin.size().width; i++)
	{
		y = 0.0;

		for (int j = 0; j < coefficient.size().height; j++)
		{
			y += coefficient.at<double>(j, 0) * pow((double)i, (double)j);
		}

		if ((int)y >= origin.size().height) y = origin.size().height - 1;
		else if ((int)y <= 0) y = 0;

		for (int k = 0; k < origin.size().height; k++)
		{
			if (origin.at<uchar>(k, i) != 0)
			{
				if (k <= (int)y)
				{
					output_Result.at<uchar>(k, i) = 255;

					temp_Point = Point(i, k);

					if ((k == (int)y) && (temp_Point.x > s_Point.x))
					{
						s_Point = temp_Point;
					}

				}
			}

		}
	}

	wrist = s_Point;


	return output_Result;
}

Mat Check_Points(Mat origin, vector<Point> dots)
{
	Mat copy = origin.clone();

	if (dots.size() != 0)
	{
		for (int i = 0; i < dots.size(); i++)
		{
			copy.at<Vec3b>(dots[i].y, dots[i].x)[0] = 255;
			copy.at<Vec3b>(dots[i].y, dots[i].x)[1] = 255;
			copy.at<Vec3b>(dots[i].y, dots[i].x)[2] = 255;
		}
	}

	return copy;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

/// <summary>
/// Constructor
/// </summary>
CBodyBasics::CBodyBasics() :
m_hWnd(NULL),
m_nStartTime(0),
m_nLastCounter(0),
m_nFramesSinceUpdate(0),
m_fFreq(0),
m_nNextStatusTime(0LL),
m_pKinectSensor(NULL),
m_pCoordinateMapper(NULL),
m_pColorFrameReader(NULL),
m_pBodyFrameReader(NULL),
m_pDepthCoordinates(NULL),
m_pD2DFactory(NULL),
m_pRenderTarget(NULL),
m_pBrushJointTracked(NULL),
m_pBrushJointInferred(NULL),
m_pBrushBoneTracked(NULL),
m_pBrushBoneInferred(NULL),
m_pBrushHandClosed(NULL),
m_pBrushHandOpen(NULL),
m_pBrushHandLasso(NULL),
m_pDepthFrameReader(NULL),
//m_pDrawDepth(NULL),
//m_pDepthRGBX(NULL),
//m_pDrawColor(NULL),
m_pColorRGBX(NULL),
m_pBackgroundRGBX(NULL),
m_pOutputRGBX(NULL)

{
	LARGE_INTEGER qpf = { 0 };
	if (QueryPerformanceFrequency(&qpf))
	{
		m_fFreq = double(qpf.QuadPart);
	}

	// create heap storage for depth pixel data in RGBX format
	//m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];

	// create heap storage for color pixel data in RGBX format
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];

	// create heap storage for the coorinate mapping from color to depth
	m_pDepthCoordinates = new DepthSpacePoint[cColorWidth * cColorHeight];

	// create heap storage for background image pixel data in RGBX format
	m_pBackgroundRGBX = new RGBQUAD[cColorWidth * cColorHeight];

	// create heap storage for composite image pixel data in RGBX format
	m_pOutputRGBX = new RGBQUAD[cColorWidth * cColorHeight];
}


/// <summary>
/// Destructor
/// </summary>
CBodyBasics::~CBodyBasics()
{
	DiscardDirect2DResources();

	// clean up Direct2D
	SafeRelease(m_pD2DFactory);

	// done with body frame reader
	SafeRelease(m_pBodyFrameReader);

	// done with coordinate mapper
	SafeRelease(m_pCoordinateMapper);

	// clean up Direct2D renderer
	//if (m_pDrawDepth)
	//{
	//	delete m_pDrawDepth;
	//	m_pDrawDepth = NULL;
	//}

	/*if (m_pDepthRGBX)
	{
	delete[] m_pDepthRGBX;
	m_pDepthRGBX = NULL;
	}*/

	//if (m_pDrawColor)
	//{
	//	delete m_pDrawColor;
	//	m_pDrawColor = NULL;
	//}

	if (m_pColorRGBX)
	{
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}

	if (m_pDepthCoordinates)
	{
		delete[] m_pDepthCoordinates;
		m_pDepthCoordinates = NULL;
	}

	if (m_pBackgroundRGBX)
	{
		delete[] m_pBackgroundRGBX;
		m_pBackgroundRGBX = NULL;
	}

	if (m_pOutputRGBX)
	{
		delete[] m_pOutputRGBX;
		m_pOutputRGBX = NULL;
	}

	// done with depth frame reader
	SafeRelease(m_pDepthFrameReader);

	// done with color frame reader
	SafeRelease(m_pColorFrameReader);

	// close the Kinect Sensor
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CBodyBasics::Run()
{

	InitializeDefaultSensor();

	string filename;
	int count = 0;

	int count_Load = 0;

	int quit = 0;

	hand_Size = 101;

	hand_Size_Color = 251;

	hand_Depth_Range = 150;

	// Main message loop
	//while (quit == 0)
	{
		show_Right_Hand = Mat::zeros(hand_Size_Color, hand_Size_Color, CV_8U);

		show_Left_Hand = Mat::zeros(hand_Size_Color, hand_Size_Color, CV_8U);

		left_Hand_Color = Mat::zeros(hand_Size_Color, hand_Size_Color, CV_8UC4);

		right_Hand_Color = Mat::zeros(hand_Size_Color, hand_Size_Color, CV_8UC4);

		left_Hand_Color_In_Depth = Mat::zeros(hand_Size_Color, hand_Size_Color, CV_32F);

		right_Hand_Color_In_Depth = Mat::zeros(hand_Size_Color, hand_Size_Color, CV_32F);

		Mat left_Hand_Color_In_Depth_view = Mat::zeros(hand_Size_Color, hand_Size_Color, CV_8U);

		Mat right_Hand_Color_In_Depth_view = Mat::zeros(hand_Size_Color, hand_Size_Color, CV_8U);

		OpenCV_View = Mat::zeros(cDepthHeight, cDepthWidth, CV_8UC3);


		Mat angle_Left = Mat::zeros(show_Left_Hand.size(), CV_8U);

		Mat angle_Right = Mat::zeros(show_Right_Hand.size(), CV_8U);

		Mat angle_Right_2 = Mat::zeros(show_Right_Hand.size(), CV_8U);

		Mat angle_Left_2 = Mat::zeros(show_Left_Hand.size(), CV_8U);

		Mat edge_Right = Mat::zeros(show_Right_Hand.size(), CV_8U);

		Mat edge_Left = Mat::zeros(show_Left_Hand.size(), CV_8U);

		Mat left_Median, right_Median, left_Resize, right_Resize, crop_Left, crop_Right, left_Hand_Color_3b, right_Hand_Color_3b;

		vector<save_Point> tracking_Right;

		vector<save_Point> tracking_Left;

		vector<Point> find_Belt_Left;

		vector<Point> find_Belt_Right;




		Update();

		imshow("test", OpenCV_View);

		//imshow("Color", color_View);

		//imshow("test2", Set_Gray_From_Depth(depth_View, depth_Data));

		//imshow("Mapping", RGB_Depth_Map);

		//imshow("test3", Set_Gray_From_Depth(depth_View_Color, depth_Data_Color));

		Cut_Hand_Color(color_View, left_Hand_Color, right_Hand_Color, L_Hand_Color, R_Hand_Color, hand_Size_Color);

		imshow("Left Color", left_Hand_Color);

		imshow("Right Color", right_Hand_Color);

		Cut_Hand_Color_Float(depth_Data_Color, left_Hand_Color_In_Depth, right_Hand_Color_In_Depth, L_Hand_Color, R_Hand_Color, jointPoints_For_Hand, hand_Size_Color, 150.0);

		//imshow("Left Color_In_Depth", Set_Gray_From_Depth(left_Hand_Color_In_Depth_view, left_Hand_Color_In_Depth));

		//imshow("Right Color_In_Depth", Set_Gray_From_Depth(right_Hand_Color_In_Depth_view, right_Hand_Color_In_Depth));

		cvtColor(left_Hand_Color, left_Hand_Color_3b, CV_BGRA2BGR);

		cvtColor(right_Hand_Color, right_Hand_Color_3b, CV_BGRA2BGR);

		find_Belt_Left = Find_Largest_Class(Find_Belt(left_Hand_Color_3b, left_Hand_Color_In_Depth));

		find_Belt_Right = Find_Largest_Class(Find_Belt(right_Hand_Color_3b, right_Hand_Color_In_Depth));

		imshow("Check_Point_Left", Check_Points(left_Hand_Color_3b, find_Belt_Left));

		imshow("Check_Point_Right", Check_Points(right_Hand_Color_3b, find_Belt_Right));

		//imshow("Ransac_Left", Show_Graph(left_Hand_Color, RANSAC_Test(find_Belt_Left, 5, 10, 100)));

		imshow("Linear_Regression_Left", Show_Graph(left_Hand_Color_3b, Linear_Regression(find_Belt_Left, 1)));

		imshow("Linear_Regression_Right", Show_Graph(right_Hand_Color_3b, Linear_Regression(find_Belt_Right, 1)));

		medianBlur(right_Hand_Color_In_Depth, right_Median, 5);

		medianBlur(left_Hand_Color_In_Depth, left_Median, 5);

		//GaussianBlur(right_Hand_Color_In_Depth, right_Median, Size(5, 5), 1.0);

		//GaussianBlur(left_Hand_Color_In_Depth, left_Median, Size(5, 5), 1.0);

		//right_Median = right_Hand_Color_In_Depth.clone();

		//left_Median = left_Hand_Color_In_Depth.clone();

		Set_Gray_From_Depth(show_Left_Hand, left_Median);

		Set_Gray_From_Depth(show_Right_Hand, right_Median);

		crop_Left = Crop_Regressiobn(show_Left_Hand, Linear_Regression(find_Belt_Left, 1), left_For_Edge);

		crop_Right = Crop_Regressiobn(show_Right_Hand, Linear_Regression(find_Belt_Right, 1), right_For_Edge);

		//ellipse(crop_Left, Point(left_For_Edge.x, left_For_Edge.y), Size(20, 20), 0, 0, 360, Scalar(0, 0, 0), 1);

		imshow("Crop_Regression_Left", crop_Left);

		imshow("Crop_Regression_Right", crop_Right);

		imshow("Left Color_In_Depth", show_Left_Hand);

		imshow("Right Color_In_Depth", show_Right_Hand);

		edge_Right = Hysteresis_Edge_Tracking_Recursive(Double_Thresholding(Non_Maximum_Suppression_Gray(Sobel_Mask_Gray(crop_Right, angle_Right_2), angle_Right_2, 100), 25, 90), 30, 255);

		edge_Left = Hysteresis_Edge_Tracking_Recursive(Double_Thresholding(Non_Maximum_Suppression_Gray(Sobel_Mask_Gray(crop_Left, angle_Left_2), angle_Left_2, 100), 25, 90), 30, 255);

		Mat right_Dist = Chamfer_Distance_Transform_Gray(Reverse_Gray(edge_Right));

		Mat left_Dist = Chamfer_Distance_Transform_Gray(Reverse_Gray(edge_Left));

		save_Point center_Right = Find_Maximum_Pixel(right_Dist, crop_Right);

		save_Point center_Left = Find_Maximum_Pixel(left_Dist, crop_Left);

		ellipse(right_Dist, Point(center_Right.x_Point, center_Right.y_Point), Size(20, 20), 0, 0, 360, Scalar(0, 0, 0), 1);

		ellipse(left_Dist, Point(center_Left.x_Point, center_Left.y_Point), Size(20, 20), 0, 0, 360, Scalar(0, 0, 0), 1);

		imshow("temp_Right", Reverse_Gray(edge_Right));

		imshow("temp_Left", Reverse_Gray(edge_Left));

		tracking_Right = Edge_Tracking_Simple(edge_Right);

		tracking_Left = Edge_Tracking_Simple(edge_Left);

		imshow("Check_Right", Check_Tracking(edge_Right, tracking_Right));

		imshow("Check_Left", Check_Tracking(edge_Left, tracking_Left));

		imshow("Right Dist", right_Dist);

		imshow("Left Dist", left_Dist);

		recent_Left.clear();

		recent_Right.clear();

		Edge_Rotation(tracking_Left, left_For_Edge, center_Left, count, 140, 80, format("Left_Show"), recent_Left);

		Edge_Rotation(tracking_Right, right_For_Edge, center_Right, count, 140, 80, format("Right_Show"), recent_Right);

		finger_Result_Left.clear();

		finger_Result_Right.clear();

		imshow("Compare_FEMD_Left", Compare_FEMD(base_Left, recent_Left, finger_Result_Left));

		imshow("Compare_FEMD_Right", Compare_FEMD(base_Right, recent_Right, finger_Result_Right));

		//if (waitKey(10) == 'q')
		//{
		//	quit = 1;
		//}
		if (waitKey(10) == 't')
		{
			base_Left.clear();

			imshow("Left_FEMD_Base", Set_FEMD_Base(recent_Left, base_Left));
		}
		else if (waitKey(10) == 'y')
		{
			base_Right.clear();

			imshow("Right_FEMD_Base", Set_FEMD_Base(recent_Right, base_Right));
		}
		else if (waitKey(10) == 'c')
		{
			//filename = format("L_Hand_Color_%03d.jpg", count);
			//imwrite(filename.c_str(), left_Hand_Color);

			/*
			// If you want to save hand depth data
			filename = format("L_Hand_Depth_Color_Size_%03d.xml", count);
			FileStorage fs1(filename.c_str(), FileStorage::WRITE);
			fs1 << "Left_Hand" << left_Hand_Color_In_Depth;
			fs1.release();
			*/
			filename = format("Left_Hand_%03d.txt", count);
			Check_Value_Gray(left_Dist, filename);

			count++;
		}
	}

	return 0;
}

/// <summary>
/// Main processing function
/// </summary>
void CBodyBasics::Update()
{


	if (!m_pBodyFrameReader)
	{
		return;
	}
	if (!m_pDepthFrameReader)
	{
		return;
	}
	if (!m_pColorFrameReader)
	{
		return;
	}

	IBodyFrame* pBodyFrame = NULL;

	IDepthFrame* pDepthFrame = NULL;

	IColorFrame* pColorFrame = NULL;

	HRESULT hr1 = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

	HRESULT hr2 = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

	HRESULT hr3 = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

	if (SUCCEEDED(hr1) && SUCCEEDED(hr2) && SUCCEEDED(hr3))
		//if (SUCCEEDED(hr1) && SUCCEEDED(hr2))
	{
		INT64 nTime = 0;

		IFrameDescription* pFrameDescription = NULL;
		IFrameDescription* pFrameDescription_color = NULL;
		int nWidth = 0;
		int nHeight = 0;
		int nWidth_color = 0;
		int nHeight_color = 0;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxDistance = 0;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nBufferSize = 0;
		UINT nBufferSize_color = 0;
		UINT16 *pBuffer = NULL;
		RGBQUAD *pBuffer_color = NULL;

		hr1 = pBodyFrame->get_RelativeTime(&nTime);

		hr2 = pDepthFrame->get_RelativeTime(&nTime);

		hr3 = pColorFrame->get_RelativeTime(&nTime);

		IBody* ppBodies[BODY_COUNT] = { 0 };

		if (SUCCEEDED(hr1) && SUCCEEDED(hr2) && SUCCEEDED(hr3))
			//if (SUCCEEDED(hr1) && SUCCEEDED(hr2))
		{
			hr1 = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);

			hr2 = pDepthFrame->get_FrameDescription(&pFrameDescription);

			hr3 = pColorFrame->get_FrameDescription(&pFrameDescription_color);

			if (SUCCEEDED(hr2))
			{
				hr2 = pFrameDescription->get_Width(&nWidth);
			}

			if (SUCCEEDED(hr2))
			{

				hr2 = pFrameDescription->get_Height(&nHeight);
			}

			if (SUCCEEDED(hr2))
			{

				hr2 = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
			}

			if (SUCCEEDED(hr2))
			{

				// In order to see the full range of depth (including the less reliable far field depth)
				// we are setting nDepthMaxDistance to the extreme potential depth threshold
				//nDepthMaxDistance = USHRT_MAX;

				// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
				hr2 = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
			}

			if (SUCCEEDED(hr2))
			{

				hr2 = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
			}

			if (SUCCEEDED(hr3))
			{
				hr3 = pColorFrame->get_FrameDescription(&pFrameDescription_color);
			}

			if (SUCCEEDED(hr3))
			{
				hr3 = pFrameDescription_color->get_Width(&nWidth_color);
			}

			if (SUCCEEDED(hr3))
			{
				hr3 = pFrameDescription_color->get_Height(&nHeight_color);
			}

			if (SUCCEEDED(hr3))
			{
				hr3 = pColorFrame->get_RawColorImageFormat(&imageFormat);
			}

			if (SUCCEEDED(hr3))
			{
				if (imageFormat == ColorImageFormat_Bgra)
				{
					hr3 = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize_color, reinterpret_cast<BYTE**>(&pBuffer_color));
				}
				else if (m_pColorRGBX)
				{
					pBuffer_color = m_pColorRGBX;
					nBufferSize_color = cColorWidth * cColorHeight * sizeof(RGBQUAD);
					hr3 = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize_color, reinterpret_cast<BYTE*>(pBuffer_color), ColorImageFormat_Bgra);
				}
				else
				{
					hr3 = E_FAIL;
				}
			}

		}

		if (SUCCEEDED(hr1) && SUCCEEDED(hr2) && SUCCEEDED(hr3))
			//if (SUCCEEDED(hr1) && SUCCEEDED(hr2))
		{
			ProcessBody(nTime, BODY_COUNT, ppBodies);

			ProcessDepth(nTime, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);

			ProcessColor(nTime, pBuffer_color, nWidth_color, nHeight_color);

			RGB_Depth_Mapping(pBuffer, nWidth, nHeight, pBuffer_color, nWidth_color, nHeight_color, color_View, depth_Data);
		}

		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);
		}

		SafeRelease(pFrameDescription);

		SafeRelease(pFrameDescription_color);

	}

	SafeRelease(pBodyFrame);

	SafeRelease(pDepthFrame);

	SafeRelease(pColorFrame);

}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CBodyBasics::InitializeDefaultSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get coordinate mapper and the body reader
		IBodyFrameSource* pBodyFrameSource = NULL;

		IDepthFrameSource* pDepthFrameSource = NULL;

		IColorFrameSource* pColorFrameSource = NULL;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
		}

		SafeRelease(pBodyFrameSource);

		SafeRelease(pDepthFrameSource);

		SafeRelease(pColorFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		//SetStatusMessage(L"No ready Kinect found!", 10000, true);
		return E_FAIL;
	}

	return hr;
}


/// <summary>
/// Handle new body data
/// <param name="nTime">timestamp of frame</param>
/// <param name="nBodyCount">body data count</param>
/// <param name="ppBodies">body data in frame</param>
/// </summary>
void CBodyBasics::ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies)
{
	//if (m_hWnd)
	{
		//HRESULT hr = EnsureDirect2DResources();

		HRESULT hr;

		if (m_pCoordinateMapper)
		{
			//m_pRenderTarget->BeginDraw();
			//m_pRenderTarget->Clear();

			RECT rct;
			//GetClientRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rct);
			//int width = rct.right;
			//int height = rct.bottom;

			Point center;
			Size radius(3, 3);

#pragma omp parallel for
			for (int i = 0; i < nBodyCount; ++i)
			{
				//cout << "test" << endl;

				IBody* pBody = ppBodies[i];
				if (pBody)
				{
					BOOLEAN bTracked = false;
					hr = pBody->get_IsTracked(&bTracked);

					if (SUCCEEDED(hr) && bTracked)
					{
						Joint joints[JointType_Count];
						D2D1_POINT_2F jointPoints[JointType_Count];
						D2D1_POINT_2F jointPoints_CV[JointType_Count];
						HandState leftHandState = HandState_Unknown;
						HandState rightHandState = HandState_Unknown;

						pBody->get_HandLeftState(&leftHandState);
						pBody->get_HandRightState(&rightHandState);

						hr = pBody->GetJoints(_countof(joints), joints);
						if (SUCCEEDED(hr))
						{
							for (int j = 0; j < _countof(joints); ++j)
							{
								//jointPoints[j] = BodyToScreen(joints[j].Position, width, height);
								//jointPoints_CV[j] = BodyToScreen(joints[j].Position, OpenCV_View.size().width, OpenCV_View.size().height);

								jointPoints_For_Hand[j] = BodyToScreen(joints[j].Position, OpenCV_View.size().width, OpenCV_View.size().height);

								center.x = jointPoints_For_Hand[j].x;
								center.y = jointPoints_For_Hand[j].y;

								center.x = jointPoints_CV[j].x;
								center.y = jointPoints_CV[j].y;

								if (j == JointType_HandLeft) ellipse(OpenCV_View, center, radius, 0, 0, 360, Scalar(255, 0, 0), 1);
								else if (j == JointType_HandRight) ellipse(OpenCV_View, center, radius, 0, 0, 360, Scalar(0, 0, 255), 1);
								else ellipse(OpenCV_View, center, radius, 0, 0, 360, Scalar(0, 255, 0), 1);
							}

							/*
							L_Hand.x = (jointPoints_CV[JointType_HandLeft].x + jointPoints_CV[JointType_HandTipLeft].x + jointPoints_CV[JointType_ThumbLeft].x) / 3;
							L_Hand.y = (jointPoints_CV[JointType_HandLeft].y + jointPoints_CV[JointType_HandTipLeft].y + jointPoints_CV[JointType_ThumbLeft].y) / 3;
							R_Hand.x = (jointPoints_CV[JointType_HandRight].x + jointPoints_CV[JointType_HandTipRight].x + jointPoints_CV[JointType_ThumbRight].x) / 3;
							R_Hand.y = (jointPoints_CV[JointType_HandRight].y + jointPoints_CV[JointType_HandTipRight].y + jointPoints_CV[JointType_ThumbRight].y) / 3;
							*/

							///*
							L_Hand.x = (int)((jointPoints_For_Hand[JointType_WristLeft].x + jointPoints_For_Hand[JointType_HandTipLeft].x) / 2);
							L_Hand.y = (int)((jointPoints_For_Hand[JointType_WristLeft].y + jointPoints_For_Hand[JointType_HandTipLeft].y) / 2);
							R_Hand.x = (int)((jointPoints_For_Hand[JointType_WristRight].x + jointPoints_For_Hand[JointType_HandTipRight].x) / 2);
							R_Hand.y = (int)((jointPoints_For_Hand[JointType_WristRight].y + jointPoints_For_Hand[JointType_HandTipRight].y) / 2);
							//*/

							ellipse(OpenCV_View, L_Hand, Size(20, 20), 0, 0, 360, Scalar(255, 0, 0), 1);
							ellipse(OpenCV_View, R_Hand, Size(20, 20), 0, 0, 360, Scalar(0, 0, 255), 1);

							//DrawBody(joints, jointPoints);

							//DrawHand(leftHandState, jointPoints[JointType_HandLeft]);
							//DrawHand(rightHandState, jointPoints[JointType_HandRight]);
						}
					}
				}
			}
		}
	}
}

/// <summary>
/// Handle new depth data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
/// <param name="nMinDepth">minimum reliable depth</param>
/// <param name="nMaxDepth">maximum reliable depth</param>
/// </summary>
void CBodyBasics::ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{

	Mat depth_data = Mat::zeros(nHeight, nWidth, CV_32F);


	// Make sure we've received valid data
	//if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
	if (pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
	{

		//RGBQUAD* pRGBX = m_pDepthRGBX;

		// end pixel is start + width*height - 1
		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

		int i = 0;

		//int temp_value = 0;

		while (pBuffer < pBufferEnd)
		{
			USHORT depth = *pBuffer;

			// To convert to a byte, we're discarding the most-significant
			// rather than least-significant bits.
			// We're preserving detail, although the intensity will "wrap."
			// Values outside the reliable depth range are mapped to 0 (black).

			// Note: Using conditionals in this loop could degrade performance.
			// Consider using a lookup table instead when writing production code.
			//BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

			//pRGBX->rgbRed = intensity;
			//pRGBX->rgbGreen = intensity;
			//pRGBX->rgbBlue = intensity;

			depth_data.at<float>(i) = (int)depth;

			i++;
			//++pRGBX;
			++pBuffer;
		}

		/*
		//Save all depth in Xml
		FileStorage fs("test.xml", FileStorage::WRITE);
		fs << "data" << depth_data;
		fs.release();
		*/

		//View depth gray
		//Set_Gray_From_Depth(depth_View, depth_data);

		depth_Data = depth_data;

		//Cut_Hand(depth_data, left_Hand, right_Hand, L_Hand, R_Hand, hand_Size, 1700);

		//Set_Gray_From_Depth(show_Left_Hand, left_Hand);

		//Set_Gray_From_Depth(show_Right_Hand, right_Hand);

		// Draw the data with Direct2D
		//m_pDrawDepth->Draw(reinterpret_cast<BYTE*>(m_pDepthRGBX), cDepthWidth * cDepthHeight * sizeof(RGBQUAD));

	}
}

void CBodyBasics::ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight)
{

	// Make sure we've received valid data
	if (pBuffer && (nWidth == cColorWidth) && (nHeight == cColorHeight))
	{
		// Draw the data with Direct2D
		//m_pDrawColor->Draw(reinterpret_cast<BYTE*>(pBuffer), cColorWidth * cColorHeight * sizeof(RGBQUAD));

		//Convert to Mat
		Mat temp(nHeight, nWidth, CV_8UC4, reinterpret_cast<void*>(pBuffer));

		//ellipse(temp, L_Hand_Color, Size(20, 20), 0, 0, 360, Scalar(255, 0, 0), 5);
		//ellipse(temp, R_Hand_Color, Size(20, 20), 0, 0, 360, Scalar(0, 0, 255), 5);

		color_View = temp;
	}
}

UINT16* Convert_Zero_Value(const UINT16 *pDepthBuffer, int nDepthWidth, int nDepthHeight)
{
	//Mat depth_test = Mat::zeros(nDepthHeight, nDepthWidth, CV_32F);

	//Mat d_view = Mat::zeros(nDepthHeight, nDepthWidth, CV_8U);

	UINT16* pDepthBuffer_Nonzero;

	pDepthBuffer_Nonzero = new UINT16[nDepthWidth * nDepthHeight];

	const UINT16* pBufferEnd = pDepthBuffer + (nDepthWidth * nDepthHeight);

	int i = 0;

	while (pDepthBuffer < pBufferEnd)
	{

		USHORT depth = *pDepthBuffer;


		pDepthBuffer_Nonzero[i] = 5000;

		++pDepthBuffer;

		i++;
	}

	return pDepthBuffer_Nonzero;

}

void CBodyBasics::RGB_Depth_Mapping(const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight, const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight, Mat color_View, Mat d_Data)
{

	//Mat RGB_Depth = Mat::zeros(nColorHeight, nColorWidth, CV_8UC4);

	Mat temp = Mat::zeros(nDepthHeight, nDepthWidth, CV_8UC4);

	Mat temp_C = Mat::zeros(nColorHeight, nColorWidth, CV_32F);

	Vec4b temp_Frame_C4;

	// Make sure we've received valid data
	if (m_pDepthCoordinates && m_pOutputRGBX &&
		pDepthBuffer && (nDepthWidth == cDepthWidth) && (nDepthHeight == cDepthHeight) &&
		pColorBuffer && (nColorWidth == cColorWidth) && (nColorHeight == cColorHeight)
		)
	{
		//UINT16* pDepthBuffer_Nonzero = Convert_Zero_Value(pDepthBuffer, nDepthWidth, nDepthHeight);

		HRESULT hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(nDepthWidth * nDepthHeight, (UINT16*)pDepthBuffer, nColorWidth * nColorHeight, m_pDepthCoordinates);
		if (SUCCEEDED(hr))
		{
#pragma omp parallel for
			// loop over output pixels
			for (int colorIndex = 0; colorIndex < (nColorWidth*nColorHeight); ++colorIndex)
			{
				// default setting source to copy from the background pixel
				//const RGBQUAD* pSrc = m_pBackgroundRGBX + colorIndex;

				DepthSpacePoint p = m_pDepthCoordinates[colorIndex];

				// Values that are negative infinity means it is an invalid color to depth mapping so we
				// skip processing for this pixel
				//if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
				{
					int depthX = static_cast<int>(p.X + 0.5f);
					int depthY = static_cast<int>(p.Y + 0.5f);

					if ((depthX >= 0 && depthX < nDepthWidth) && (depthY >= 0 && depthY < nDepthHeight))
					{
						//pSrc = m_pColorRGBX + colorIndex;

						temp_Frame_C4 = ((Vec4b*)m_pColorRGBX)[colorIndex];

						//temp.at<Vec4b>(depthY, depthX) = color_View.at<Vec4b>((colorIndex / nColorWidth), (colorIndex % nColorWidth));

						temp.at<Vec4b>(depthY, depthX) = temp_Frame_C4;

						temp_C.at<float>(colorIndex) = d_Data.at<float>(depthY, depthX);


						if ((L_Hand.x == depthX) && (L_Hand.y == depthY))
						{
							L_Hand_Color.x = colorIndex % nColorWidth;

							L_Hand_Color.y = colorIndex / nColorWidth;
						}
						if ((R_Hand.x == depthX) && (R_Hand.y == depthY))
						{
							R_Hand_Color.x = colorIndex % nColorWidth;

							R_Hand_Color.y = colorIndex / nColorWidth;
						}


					}
				}

				// write output
				//m_pOutputRGBX[colorIndex] = *pSrc;
			}
			// Draw the data with Direct2D
			//m_pDrawCoordinateMapping->Draw(reinterpret_cast<BYTE*>(m_pOutputRGBX), cColorWidth * cColorHeight * sizeof(RGBQUAD));

		}
	}

	//Mat RGB_Depth(nColorHeight, nColorWidth, CV_8UC4, reinterpret_cast<void*>(m_pOutputRGBX));

	//RGB_Depth_Map = RGB_Depth;

	/*for (int i = 0; i < temp.size().height; i++)
	{
	for (int j = 0; j < temp.size().width; j++)
	{
	temp_C3.at<Vec3b>(i, j)[0] = temp.at<Vec4b>(i, j)[0];
	temp_C3.at<Vec3b>(i, j)[1] = temp.at<Vec4b>(i, j)[1];
	temp_C3.at<Vec3b>(i, j)[2] = temp.at<Vec4b>(i, j)[2];
	}
	}*/

	RGB_Depth_Map = temp;

	depth_Data_Color = temp_C;

	//imshow("m", temp);

}

/// <summary>
/// Dispose Direct2d resources 
/// </summary>
void CBodyBasics::DiscardDirect2DResources()
{
	SafeRelease(m_pRenderTarget);

	SafeRelease(m_pBrushJointTracked);
	SafeRelease(m_pBrushJointInferred);
	SafeRelease(m_pBrushBoneTracked);
	SafeRelease(m_pBrushBoneInferred);

	SafeRelease(m_pBrushHandClosed);
	SafeRelease(m_pBrushHandOpen);
	SafeRelease(m_pBrushHandLasso);
}

/// <summary>
/// Converts a body point to screen space
/// </summary>
/// <param name="bodyPoint">body point to tranform</param>
/// <param name="width">width (in pixels) of output buffer</param>
/// <param name="height">height (in pixels) of output buffer</param>
/// <returns>point in screen-space</returns>
D2D1_POINT_2F CBodyBasics::BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height)
{
	// Calculate the body's position on the screen
	DepthSpacePoint depthPoint = { 0 };
	m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

	float screenPointX = static_cast<float>(depthPoint.X * width) / cDepthWidth;
	float screenPointY = static_cast<float>(depthPoint.Y * height) / cDepthHeight;

	return D2D1::Point2F(screenPointX, screenPointY);
}

// --------------------------------------------------------------------------
// 
//	From Kinect Source End
// 
// --------------------------------------------------------------------------
