// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/Actor.h"
#include <vector>
THIRD_PARTY_INCLUDES_START
#include "../ThirdParty/TsMagicHelper/TsMagicHelper/TsMagicHelper.h"
THIRD_PARTY_INCLUDES_END

#include <Runtime\Engine\Classes\Engine\Engine.h>

#include "AimHelper.generated.h"

using namespace TSMagicUtil;
/**
 * 
 */
UCLASS()
class FILTERALGORITHM_API UAimHelper : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

	// we need to construct two rotation matrix, one for base one for where gun pipe is pointing at...
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Aim Function Library")
	static FMatrix CalculateBaseAimPoint(const FVector& EyePosition, const FVector& LookAtPosition, const FVector& UpVector)
	{
		// 把LookAt Position 和 Eye Position 投放到一个平面(X,Y plane)
		FVector TargetPosition = FVector(LookAtPosition.X, LookAtPosition.Y, EyePosition.Z);

		TSMagicUtil::TSMathUtil* TSMathUtilPtr = new TSMagicUtil::TSMathUtil();

		TSMagicUtil::TSMatrix LookAtMatrix = TSMathUtilPtr->ConstructUE4LookAtMatrix(TSMagicUtil::TSVector(EyePosition.X, EyePosition.Y, EyePosition.Z),
			TSMagicUtil::TSVector(TargetPosition.X, TargetPosition.Y, TargetPosition.Z),
			TSMagicUtil::TSVector(UpVector.X, UpVector.Y, UpVector.Z));

		return ConstructMatrix(LookAtMatrix.M);
	}


	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Aim Function Libary")
	static FMatrix CalcualteGunAimPoint(const FTransform& BaseTransform, const FVector& EyePosition, const FVector& LookAtPosition, const FVector& UpVector)
	{
		// 获取枪管上一个节点的Transform, 并且转成矩阵
		FMatrix TransformMatrix = BaseTransform.ToMatrixNoScale();

		// 通过 World Transform 矩阵 转换成 四元素
		FQuat RotationMatrix = FQuat(TransformMatrix);

		// 把当前Position 信息 和 四元素相乘， 获取新的枪管位置
		FVector NewEyePosition = RotationMatrix * EyePosition;

		// 构造LookAt, 因为考虑到上个节点 base 的 X,Y Plane rotation 所以这里就算构造出 3D rotation 对应也应该只是 relative roll rotation.
		TSMagicUtil::TSMathUtil* TSMathUtilPtr = new TSMagicUtil::TSMathUtil();

		TSMagicUtil::TSMatrix LookAtMatrix = TSMathUtilPtr->ConstructUE4LookAtMatrix(TSMagicUtil::TSVector(NewEyePosition.X, NewEyePosition.Y, NewEyePosition.Z),
			TSMagicUtil::TSVector(LookAtPosition.X, LookAtPosition.Y, LookAtPosition.Z),
			TSMagicUtil::TSVector(UpVector.X, UpVector.Y, UpVector.Z));

		return ConstructMatrix(LookAtMatrix.M);
	}


	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Aim Function Library")
	static FVector ContiniousOrbitAroundPoint(const FVector& CenterPosition, const float& Radius, float t)
	{
		TSVector CalculatedPos = TSMagicUtil::TSSplineUtil::CircularCalculation(TSVector(CenterPosition.X, CenterPosition.Y, CenterPosition.Z), Radius, t);


		return FVector(CalculatedPos.X, CalculatedPos.Y, CalculatedPos.Z);
	}




	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Aim Function Libary")
	// 只需要叫一次， 存在蓝图里 的 Noise
	static TArray<float> GeneratePerlinNoise(const int& Resolution, const float& ErrorAmplitude, const float& Frequency)
	{
		TArray<float> PerlinData;

		std::vector<float> Gradient1D = { -1, 0, 1 };

		// static hash for random numbers 
		int Permutation[] = {
		151,160,137, 91, 90, 15,131, 13,201, 95, 96, 53,194,233,  7,225,
		140, 36,103, 30, 69,142,  8, 99, 37,240, 21, 10, 23,190,  6,148,
		247,120,234, 75,  0, 26,197, 62, 94,252,219,203,117, 35, 11, 32,
		 57,177, 33, 88,237,149, 56, 87,174, 20,125,136,171,168, 68,175,
		 74,165, 71,134,139, 48, 27,166, 77,146,158,231, 83,111,229,122,
		 60,211,133,230,220,105, 92, 41, 55, 46,245, 40,244,102,143, 54,
		 65, 25, 63,161,  1,216, 80, 73,209, 76,132,187,208, 89, 18,169,
		200,196,135,130,116,188,159, 86,164,100,109,198,173,186,  3, 64,
		 52,217,226,250,124,123,  5,202, 38,147,118,126,255, 82, 85,212,
		207,206, 59,227, 47, 16, 58, 17,182,189, 28, 42,223,183,170,213,
		119,248,152,  2, 44,154,163, 70,221,153,101,155,167, 43,172,  9,
		129, 22, 39,253, 19, 98,108,110, 79,113,224,232,178,185,112,104,
		218,246, 97,228,251, 34,242,193,238,210,144, 12,191,179,162,241,
		 81, 51,145,235,249, 14,239,107, 49,192,214, 31,181,199,106,157,
		184, 84,204,176,115,121, 50, 45,127,  4,150,254,138,236,205, 93,
		222,114, 67, 29, 24, 72,243,141,128,195, 78, 66,215, 61,156,180 };

		int n = sizeof(Permutation) / sizeof(Permutation[0]);

		std::vector<int> HashVector(Permutation, Permutation + n);

		for (int i = 0; i < Resolution; i++)
		{
			float Noise = TSMagicUtil::TSNoiseUtil::Perlin1D((float)i/(float)Resolution * 5, Frequency, HashVector, 255, Gradient1D, 3); 

			PerlinData.Add(Noise* 2);
			
		}

		return PerlinData;
	}

	UFUNCTION(BlueprintCallable, Category = "Aim Function Library")
	// 柏林噪声 生成雷达测试性测量数据
	static FVector FakeMeasurementUsingNoise(const TArray<float> PerlinData, const FVector& RealPosition, float ErrorMargin, int Counter)
	{
		int Size = TSMagicUtil::TSMathUtil::NearestPow2Low(PerlinData.Num()) - 1;

		int XCounter = Counter;
		int YCounter = Counter + 125;
		int ZCounter = Counter + 256;

		float NoiseX = PerlinData[XCounter & Size] * ErrorMargin;
		float NoiseY = PerlinData[YCounter & Size] * ErrorMargin;
		float NoiseZ = PerlinData[ZCounter & Size] * ErrorMargin;

		FVector MeasurementPos = RealPosition + FVector(NoiseX, NoiseY, NoiseZ);

		return MeasurementPos;
	}

	UFUNCTION(BlueprintCallable, Category = "Aim Function Library", meta = (WorldContext = "WorldContextObject"))
	static void DebugIn3DWorld(UObject* WorldContextObject, const FVector& MeasurementPos, const FVector& EyePosition, const FVector& LSEstimatePos, const FVector& KMEstimatePos)
	{
		UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);

		if (World != nullptr)
		{
			DrawDebugSphere(World, MeasurementPos, 100, 100, FColor::Red);
			DrawDebugLine(World, EyePosition, MeasurementPos, FColor::Red);

			DrawDebugSphere(World, LSEstimatePos, 100, 100, FColor::Green);
			DrawDebugLine(World, EyePosition, LSEstimatePos, FColor::Green);

			DrawDebugSphere(World, KMEstimatePos, 100, 100, FColor::Blue);
			DrawDebugLine(World, EyePosition, KMEstimatePos, FColor::Blue);
		}
	}

	static FMatrix ConstructMatrix(float M[4][4])
	{
		FMatrix TempMatrix = FMatrix();
		
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				TempMatrix.M[i][j] = M[i][j];
			}
		}
		return TempMatrix;
	}

};
