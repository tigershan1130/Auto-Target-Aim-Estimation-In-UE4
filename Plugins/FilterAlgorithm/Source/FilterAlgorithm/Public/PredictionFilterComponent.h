// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
THIRD_PARTY_INCLUDES_START
#include "../ThirdParty/TsMagicHelper/TsMagicHelper/TsMagicHelper.h"
THIRD_PARTY_INCLUDES_END


#include "PredictionFilterComponent.generated.h"

//Just For Get Info 
USTRUCT(BlueprintType)
struct FLeastSquarePrediction
{
	GENERATED_BODY()
public:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float SumY;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float SumX;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float SumXX;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float SumXY;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MeanY;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MeanX;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float Slope;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float a;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float CurrentValue;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float CurrentTime;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	int N;

	FLeastSquarePrediction() {}

	FLeastSquarePrediction(float sumY, float sumX, float sumXX, float sumXY, float meanY, float meanX, float slope, float a, float currentValue, float currentTime, int n) :
		SumY(sumY), SumX(sumX), SumXX(sumXX), SumXY(sumXY), MeanY(meanY), MeanX(meanX), Slope(slope), a(a), CurrentValue(currentValue), CurrentTime(currentTime), N(n)
	{}
};


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class FILTERALGORITHM_API UPredictionFilterComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UPredictionFilterComponent();

protected:
	TSMagicUtil::TSLeastSquarePrediction* LSPredictionX = nullptr;
	TSMagicUtil::TSLeastSquarePrediction* LSPredictionY = nullptr;
	TSMagicUtil::TSLeastSquarePrediction* LSPredictionZ = nullptr;

	TSMagicUtil::TSKalmannFilterPrediction* KMPredictionX = nullptr;
	TSMagicUtil::TSKalmannFilterPrediction* KMPredictionY = nullptr;
	TSMagicUtil::TSKalmannFilterPrediction* KMPredictionZ = nullptr;

	// Called when the game starts
	virtual void BeginPlay() override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:	

	UFUNCTION(BlueprintCallable, Category = "Prediction Filter")
		void UpdateLeastSquarePredction(const FVector& Position, const float& TimeStamp);
		
	UFUNCTION(BlueprintCallable, Category = "Prediction Filter")
		FVector LeastSquareEstimatePosition(const float& PredictionTime);

	UFUNCTION(BlueprintCallable, Category = "Prediction Filter")
		FVector LeastSquareEstimateVelocity();

	//TODO: Get Values from TS struct
	UFUNCTION(BlueprintCallable, Category = "Prediction Filter")
		FLeastSquarePrediction GetLeastSquarePredictionValue1D(int direction)
	{
		FLeastSquarePrediction result;
		switch (direction)
		{
		case 0:
			result = FLeastSquarePrediction(LSPredictionX->SumY, LSPredictionX->SumX, LSPredictionX->SumXX, LSPredictionX->SumXY, LSPredictionX->MeanY, LSPredictionX->MeanX, LSPredictionX->Slope, LSPredictionX->a, LSPredictionX->CurrentValue, LSPredictionX->CurrentTime, LSPredictionX->N);
			break;
		case 1:
			result = FLeastSquarePrediction(LSPredictionY->SumY, LSPredictionY->SumX, LSPredictionY->SumXX, LSPredictionY->SumXY, LSPredictionY->MeanY, LSPredictionY->MeanX, LSPredictionY->Slope, LSPredictionY->a, LSPredictionY->CurrentValue, LSPredictionY->CurrentTime, LSPredictionY->N);
			break;
		case 2:
			result = FLeastSquarePrediction(LSPredictionZ->SumY, LSPredictionZ->SumX, LSPredictionZ->SumXX, LSPredictionZ->SumXY, LSPredictionZ->MeanY, LSPredictionZ->MeanX, LSPredictionZ->Slope, LSPredictionZ->a, LSPredictionZ->CurrentValue, LSPredictionZ->CurrentTime, LSPredictionZ->N);
			break;
		default:
			break;
		}
		return result;
	}
	/// <summary>
	/// 卡尔曼滤波构造函数
	/// </summary>
	/// <param name="mR">对方就跟我说R 是常数...280</param>
	/// <param name="mQ">也是常数....5，也许是误差？</param>
	/// <param name="mT">deltatime/时间间隔</param>
	/// <param name="x0">start Position, 开始位置</param>
	/// <param name="v0">start Speed, 初始速度</param>
	UFUNCTION(BlueprintCallable, Category = "Prediction Filter")
		void InitializeKalmanFilterPrediction(float mR, float mQ, float FixedDeltaTime, FVector StartPosition, FVector StartVelocity);

	UFUNCTION(BlueprintCallable, Category = "Prediction Filter")
		// when intialize KalmanFilterPrediction we pass in fixed time, so our actual time of update is our TimeStep * FixedTime
		void UpdateKalmanFilterPrediction(const FVector& Position, const int& TimeStep);

	UFUNCTION(BlueprintCallable, Category = "Prediction Filter")
		// when Time step is actually how many time step ahead...
		FVector KalmanFilterEstimatePosition(const int& TimeStep); 

	UFUNCTION(BlueprintCallable, Category = "Prediction Filter")
		FVector KalmanFilterEstimateVelocity(const int& TimeStep);



};
