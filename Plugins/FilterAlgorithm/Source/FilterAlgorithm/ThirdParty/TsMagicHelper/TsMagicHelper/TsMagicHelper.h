#pragma once
#include <cstdint>
#include <cmath>
#include <vector>

namespace TSMagicUtil
{
	// FVector class like UE4
	struct TSVector
	{
	public:
		float X;
		float Y;
		float Z;

		TSVector()
		{
			X = 0;
			Y = 0;
			Z = 0;
		}

		TSVector(float mX, float mY, float mZ)
		{
			X = mX;
			Y = mY;
			Z = mZ;
		}

		TSVector GetNormalized()
		{
			const float SquareSum = X * X + Y * Y + Z * Z;

			const float Scale = 1 / sqrt(SquareSum);

			return TSVector(X * Scale, Y * Scale, Z * Scale);
		}
		
		TSVector operator*(float operand) const
		{
			return TSVector(operand * X, operand * Y, operand * Z);

		}

		TSVector operator*(const TSVector& V) const
		{
			return TSVector(V.X * X, V.Y * Y, V.Z * Z);
		}

		// cross product
		TSVector operator^(const TSVector& V) const
		{
			return TSVector
			(
				Y * V.Z - Z * V.Y,
				Z * V.X - X * V.Z,
				X * V.Y - Y * V.X
			);
		}

		// dot product
		float operator|(const TSVector& V) const
		{
			return X * V.X + Y * V.Y + Z * V.Z;
		}

		// vector subtraction
		TSVector operator-(const TSVector& V) const
		{
			return TSVector(X - V.X, Y - V.Y, Z - V.Z);
		}

		// vertor addition
		TSVector operator+(const TSVector& V) const
		{
			return TSVector(X + V.X, Y + V.Y, Z + V.Z);
		}

		// vertor addition
		TSVector operator+=(const TSVector& V) const
		{
			return TSVector(X + V.X, Y + V.Y, Z + V.Z);
		}

		// override negative value
		TSVector operator-() const
		{
			return TSVector(X * -1, Y * -1, Z * -1);
		}

	};

	// FMatrix class like UE4
	struct TSMatrix
	{
	public:
		// Variables.
		float M[4][4];

		TSMatrix()
		{
			SetToIdentity();
		}

		void SetMatrix(float mM[4][4])
		{
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					M[i][j] = mM[i][j];
				}
			}
		}

		void SetToIdentity()
		{
			M[0][0] = 1; M[0][1] = 0; M[0][2] = 0; M[0][3] = 0;
			M[1][0] = 0; M[1][1] = 1; M[1][2] = 0; M[1][3] = 0;
			M[2][0] = 0; M[2][1] = 0; M[2][2] = 1; M[2][3] = 0;
			M[3][0] = 0; M[3][1] = 0; M[3][2] = 0; M[3][3] = 1;
		}

		TSMatrix GetTransposed()
		{
			TSMatrix Temp = TSMatrix();

			Temp.SetMatrix(M);

			Temp.M[0][0] = M[0][0]; Temp.M[0][1] = M[1][0]; Temp.M[0][2] = M[2][0]; Temp.M[0][3] = M[3][0];
			Temp.M[1][0] = M[0][1]; Temp.M[1][1] = M[1][1]; Temp.M[1][2] = M[2][1]; Temp.M[1][3] = M[3][1];
			Temp.M[2][0] = M[0][2]; Temp.M[2][1] = M[1][2]; Temp.M[2][2] = M[2][2]; Temp.M[2][3] = M[3][2];
			Temp.M[3][0] = M[0][3]; Temp.M[3][1] = M[1][3]; Temp.M[3][2] = M[2][3]; Temp.M[3][3] = M[3][3];

			return Temp;
		}
	};

	struct TSVector2D {
	public:
		float X;
		float Y;

		TSVector2D()
		{
			X = 0;
			Y = 0;
		}

		TSVector2D(float x, float y)
		{
			X = x;
			Y = y;
		}

		TSVector2D GetNormalized()
		{
			const float SquareSum = X * X + Y * Y;

			const float Scale = 1 / sqrt(SquareSum);

			return TSVector2D(X * Scale, Y * Scale);
		}

		TSVector2D operator*(float operand) const
		{
			return TSVector2D(operand * X, operand * Y);

		}
		
		TSVector2D operator+(const TSVector2D& V) const
		{
			return TSVector2D(X + V.X, Y + V.Y);
		}

		TSVector2D operator*(const TSVector2D& V) const
		{
			return TSVector2D(V.X * X, V.Y * Y);
		}

		// dot product
		float operator|(const TSVector2D& V) const
		{
			return X * V.X + Y * V.Y;
		}

	};

	// Matrix2D for Kalmann filter-_-
	struct TSMatrix2D
	{
	public:
		float M[2][2];

		TSMatrix2D()
		{
			for (int i = 0; i < 2; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					M[i][j] = 0;
				}
			}

			SetToIdentity();
		}

		TSMatrix2D(float M00, float M01, float M10, float M11)
		{
			M[0][0] = M00;
			M[0][1] = M01;
			M[1][0] = M10;
			M[1][1] = M11;
		}

		void SetMatrix(float mM[2][2])
		{
			for (int i = 0; i < 2; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					M[i][j] = mM[i][j];
				}
			}
		}

		void SetToIdentity()
		{
			M[0][0] = 1; M[0][1] = 0;
			M[1][0] = 0; M[1][1] = 1; 
		}

		TSMatrix2D GetTransposed()
		{
			TSMatrix2D Temp = TSMatrix2D();

			Temp.SetMatrix(M);

			Temp.M[0][0] = M[0][0]; Temp.M[0][1] = M[1][0]; 
			Temp.M[1][0] = M[0][1]; Temp.M[1][1] = M[1][1]; 
			
			return Temp;

		}

		TSMatrix2D operator+(const TSMatrix2D& V) const
		{
			float tempM[2][2];

			for (int i = 0; i < 2; i++)
			{
				for (int j = 0; j < 2; j++)
				{
					tempM[i][j] = M[i][j] + V.M[i][j];

				}
			}

			TSMatrix2D matrix = TSMatrix2D();
			matrix.SetMatrix(tempM);

			return matrix;
		}

		TSMatrix2D operator*(const TSMatrix2D& V) const
		{
			float tempM[2][2];

			tempM[0][0] = M[0][0] * V.M[0][0] + M[0][1] * V.M[1][0];
			tempM[0][1] = M[0][0] * V.M[1][0] + M[0][1] * V.M[1][1];
			tempM[1][0] = M[1][0] * V.M[0][0] + M[1][1] * V.M[1][0];
			tempM[1][1] = M[1][0] * V.M[1][0] + M[1][1] * V.M[1][1];

			TSMatrix2D matrix = TSMatrix2D();
			matrix.SetMatrix(tempM);

			return matrix;
		}

		TSVector2D operator*(TSVector2D& V) const
		{
			float M00 = V.X * M[0][0] + V.Y * M[0][1];
			float M10 = V.X * M[1][0] + V.Y * M[1][1];

			return TSVector2D(M00, M10);
		}

		TSMatrix2D operator*(const float& F) const
		{
			float M00 = M[0][0] * F;
			float M01 = M[0][1] * F;
			float M10 = M[1][0] * F;
			float M11 = M[1][1] * F;


			return TSMatrix2D(M00, M01, M10, M11);
		}
	};

	// For Spline defining handles with control point.
	struct TSCurvePoint
	{
		TSVector OutTangent;

		TSVector InTangent;

		TSVector Position;

		TSCurvePoint()
		{
			OutTangent = TSVector();
			InTangent = TSVector();
			Position = TSVector();
		}

		TSCurvePoint(const TSVector& mPosition)
		{
			Position = mPosition;
			OutTangent = TSVector();
			InTangent = TSVector();
		}
	};

	class TSMathUtil
	{
	public:
		// not for DX use, it is row matrix within UE4 uses.
		static TSMatrix ConstructUE4LookAtMatrix(const TSVector& EyePosition, const TSVector& LookAtPosition, const TSVector& UpVector);
		
		static unsigned int  NearestPow2Low(unsigned int  N);

		static unsigned int  NearestPow2High(unsigned int  N);
	};

	class TSSplineUtil
	{
	public:

		// cubic bezier function 4 control points for cubic bezier control
		static TSVector CubicBezierCurve(const TSVector& P0, const TSVector& P1, const TSVector& P2, const TSVector& P3, float t);
		
		// construct quadratic bezier curve.
		static TSVector QuadraticBezierCurve(const TSVector& P0, const TSVector& P1, const TSVector& P2, float t);

		// hermite curve input two tangents to control a special cubic curve.
		static TSVector HermiteCurve(const TSVector& P0, const TSVector& P1, const TSVector& P2, const TSVector& P3, float t);
		
		// nth order bezier curve, not really used because it is very ineffcient, just here for reference.
		static TSVector NthOrderBezierCurve(std::vector<TSVector> Points, float t);

		// going around a circle.
		static TSVector CircularCalculation(TSVector origin, float radius, float time);
		
		// Binomial Cofficient is n! / (i! * (n - i)!)
		static float CalcBinomialCoefficient(int N, int i);
		
		// this will reutrn n! 
		// if n = 5, then 5! = 5 * 4 * 3 * 2 * 1;
		static float Factorial(int N);

		// this will construct multiple bezier/hermite curves into one spline with auto tangents been generated for smooth curve.
		// curve Type = 0 is bezier
		// curve type = 1 is hermite
		static std::vector<TSCurvePoint> ConstructCatmullRomSpline(std::vector<TSVector> Points, bool looped);
	};

	class TSNoiseUtil
	{
	public:

		static std::vector<int> GenerateHash(int resolution, int hashMask = 255);
		
		// this is just reading from hash without graident smoothness
		static float Value1D(float X, float Frequency, std::vector<int> Hash, int HashMask);

		// 2D 
		static float Value2D(TSVector XY, float Frequency, std::vector<int> Hash, int HashMask);


		static float Value3D(TSVector Point, float Frequency, std::vector<int> Hash, int HashMask);


		static float Perlin1D(float X, float Frequency, std::vector<int> Hash, int HashMask, std::vector<float> Gradients1D, int GradientMask);
		

		static float Perlin2D(TSVector XY, float Frequency, std::vector<int> Hash, int HashMask, std::vector<TSVector> Gradients2D, int GradientMask);


		static float Perlin3D(TSVector XY, float Frequency, std::vector<int> Hash, int HashMask, std::vector<TSVector> Gradients3D, int GradientMask);
	};

	class TSInterpolationUtil
	{
	public:
		static float LinearInterpolation(float Start, float End, float t);

		static TSVector LinearInterpolation(TSVector Start, TSVector End, float t);

		// this is also 3rd degree polynoimal 
		static float Smooth_3rdDegreePolynomial(float t);

		// 5th degree polynomials
		static float Smooth_5thDegreePolynomial(float t);

		static float SmoothStep(float edge0, float edge1, float x);
	};

	struct TSLeastSquarePrediction {


	public:
		float SumY;
		float SumX;
		float SumXX;
		float SumXY;

		float MeanY;
		float MeanX;

		float Slope;
		float a;


		float CurrentValue;
		float CurrentTime;

		int N;

		TSLeastSquarePrediction()
		{
			SumY = 0;
			SumX = 0;
			SumXX = 0;
			SumXY = 0;
			MeanY = 0;
			MeanX = 0;
			Slope = 0;
			CurrentValue = 0;
			CurrentTime = 0;
			a = 0;
			N = 0;
		}

		void ResetValues()
		{
			SumY = 0;
			SumX = 0;
			SumXX = 0;
			SumXY = 0;
			MeanY = 0;
			MeanX = 0;
			Slope = 0;
			CurrentValue = 0;
			CurrentTime = 0;
			a = 0;
			N = 0;
		}

		void UpdateData(float CurrentVal, float TimeStamp);

		float GetLSE(float time);

		// get velocity at current tangent
		float GetVelocity();

	};

	struct TSKalmannFilterPrediction
	{
	public:
		float R; //
		float Q; // 
		float T; // 时间间隔

		TSMatrix2D P1; 

		float X0; // start distance
		float V0; // Start speed

		float k;
		float xk;

		TSMatrix2D A;

		TSVector2D XV_;
		TSVector2D XV_P;

		TSMatrix2D P_;
		TSMatrix2D P_P;
		TSMatrix2D Q_P;

		TSMatrix2D KalmannGain;

		float QT2;

		std::vector<float> EstimatedDists;


		float CalculatedK;

		/// <summary>
		/// 卡尔曼滤波构造函数
		/// </summary>
		/// <param name="mR">对方就跟我说R 是常数...280</param>
		/// <param name="mQ">也是常数....5，也许是误差？</param>
		/// <param name="mT">deltatime/时间间隔</param>
		/// <param name="x0">start Position, 开始位置</param>
		/// <param name="v0">start Speed, 初始速度</param>
		TSKalmannFilterPrediction(float mR, float mQ, float mT, float x0, float v0)
		{
			R = mR;
			Q = mQ;
			T = mT;

			X0 = x0;
			V0 = v0;

			P1 = TSMatrix2D(R, R / T,
				R / T, R * 2 / (T * T));

			k = 0;
			xk = X0;

			XV_ = TSVector2D(X0, V0);
			XV_P = TSVector2D(0, 0);

			P_ = TSMatrix2D(0, 0, 0, 0);
			P_P = TSMatrix2D(0, 0, 0, 0);

			A = TSMatrix2D(1, T, 
				0, 1);

			QT2 = 0.25f * Q * (T * T);

			Q_P = TSMatrix2D(T * T * QT2, T * QT2, T * QT2, 1);

			KalmannGain = TSMatrix2D(1, 1, 1, 1);

			CalculatedK = 0;
		}


		// 书本上 Equation 4. @.@ 更新公式 P_P 距离更新矩阵？
		TSMatrix2D CalculateP_P(TSMatrix2D p_, float k1, float k2);

		// 书本上 Equation 1. @.@ 更新公式 XV_P 速度更新矩阵？
		TSVector2D CalculateXV_P(TSVector2D xv_, float k1, float k2, float measureX, float calcX);
		
		// 当新的位置进来后， 我们将新的位置加入到 measuredDist;
		// k is k = time / T
		void UpdateKalmannFilter(float CurrentValue, int k);

		float GetCurrentDist()
		{
			return XV_P.X;
		}

		float GetCurrentVel()
		{
			return XV_P.Y; //??
		}

	};

	class TSAlgorithm
	{
	public:
		static float Clamp(float v, float lo, float hi);



	};
}