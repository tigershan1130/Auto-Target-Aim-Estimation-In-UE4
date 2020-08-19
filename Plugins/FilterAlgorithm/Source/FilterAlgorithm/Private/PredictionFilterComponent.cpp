// Fill out your copyright notice in the Description page of Project Settings.


#include "PredictionFilterComponent.h"

// Sets default values for this component's properties
UPredictionFilterComponent::UPredictionFilterComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void UPredictionFilterComponent::BeginPlay()
{
	Super::BeginPlay();

	// dynamic memory allocation for least square predction
	LSPredictionX = new TSMagicUtil::TSLeastSquarePrediction();
	LSPredictionY = new TSMagicUtil::TSLeastSquarePrediction();
	LSPredictionZ = new TSMagicUtil::TSLeastSquarePrediction();

	LSPredictionX->ResetValues();
	LSPredictionY->ResetValues();
	LSPredictionZ->ResetValues();

	// ...
	
}

void UPredictionFilterComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (LSPredictionX != nullptr)
	{
		LSPredictionX = nullptr;
		// delete LSPredictionX // this cause crash
	}

	if(LSPredictionY != nullptr)
	{
		LSPredictionY = nullptr;
		// delete LSPredictionX // this cause crash
	}

	if(LSPredictionZ != nullptr)
	{
		LSPredictionZ = nullptr;
		// delete LSPredictionX // this cause crash
	}
}

void UPredictionFilterComponent::UpdateLeastSquarePredction(const FVector& Position, const float& TimeStamp)
{
	if (LSPredictionX == nullptr || LSPredictionY == nullptr || LSPredictionZ == nullptr)
		return;

	LSPredictionX->UpdateData(Position.X, TimeStamp);
	LSPredictionY->UpdateData(Position.Y, TimeStamp);
	LSPredictionZ->UpdateData(Position.Z, TimeStamp);
}

FVector UPredictionFilterComponent::LeastSquareEstimatePosition(const float& PredictionTime)
{
	if (LSPredictionX == nullptr || LSPredictionY == nullptr || LSPredictionZ == nullptr)
		return FVector(0, 0, 0);

	float X = LSPredictionX->GetLSE(PredictionTime);
	float Y = LSPredictionY->GetLSE(PredictionTime);
	float Z = LSPredictionZ->GetLSE(PredictionTime);

	return FVector(X, Y, Z);
}

FVector UPredictionFilterComponent::LeastSquareEstimateVelocity()
{
	if (LSPredictionX == nullptr || LSPredictionY == nullptr || LSPredictionZ == nullptr)
		return FVector(0, 0, 0);

	return FVector(LSPredictionX->GetVelocity(),
		LSPredictionY->GetVelocity(),
		LSPredictionZ->GetVelocity());
}

//  MR, MQ i guess is guassian noise's disburse value and noise value.
void UPredictionFilterComponent::InitializeKalmanFilterPrediction(float mR, float mQ, float FixedDeltaTime, FVector StartPosition, FVector StartVelocity)
{
	KMPredictionX = new TSMagicUtil::TSKalmannFilterPrediction(mR, mQ, FixedDeltaTime, StartPosition.X, StartVelocity.X);
	KMPredictionY = new TSMagicUtil::TSKalmannFilterPrediction(mR, mQ, FixedDeltaTime, StartPosition.Y, StartVelocity.Y);
	KMPredictionZ = new TSMagicUtil::TSKalmannFilterPrediction(mR, mQ, FixedDeltaTime, StartPosition.Z, StartVelocity.Z);

}

void UPredictionFilterComponent::UpdateKalmanFilterPrediction(const FVector& Position, const int& TimeStep)
{
	if (KMPredictionX == nullptr || KMPredictionY == nullptr || KMPredictionZ == nullptr)
		return;

	KMPredictionX->UpdateKalmannFilter(Position.X, TimeStep);
	KMPredictionY->UpdateKalmannFilter(Position.Y, TimeStep);
	KMPredictionZ->UpdateKalmannFilter(Position.Z, TimeStep);


}

FVector UPredictionFilterComponent::KalmanFilterEstimatePosition(const int& TimeStep)
{
	if (KMPredictionX == nullptr || KMPredictionY == nullptr || KMPredictionZ == nullptr)
		return FVector();

	return FVector(KMPredictionX->GetCurrentDist(), KMPredictionY->GetCurrentDist(), KMPredictionZ->GetCurrentDist());
}


FVector UPredictionFilterComponent::KalmanFilterEstimateVelocity(const int& TimeStep)
{
	if (KMPredictionX == nullptr || KMPredictionY == nullptr || KMPredictionZ == nullptr)
		return FVector();

	return FVector(KMPredictionX->GetCurrentVel(), KMPredictionY->GetCurrentVel(), KMPredictionZ->GetCurrentVel());
}




