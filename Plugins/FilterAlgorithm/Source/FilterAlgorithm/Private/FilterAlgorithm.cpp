// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

#include "FilterAlgorithm.h"

#define LOCTEXT_NAMESPACE "FFilterAlgorithmModule"

void FFilterAlgorithmModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
	//IPluginManager::Get().FindPlugin("PluginName")->GetBaseDir();

	//**DLLHandle = FPlatformProcess::GetDllHandle(Path);

}

void FFilterAlgorithmModule::ShutdownModule()
{
	//FPlatformProcess::FreeDllHandle(DLLHandle);
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FFilterAlgorithmModule, FilterAlgorithm)