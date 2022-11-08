#pragma once
#include <vector>
#include "Carla/Sensor/Sensor.h"

#include "Carla/Actor/ActorDefinition.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/sensor/data/UltrasonicData.h>
#include <compiler/enable-ue4-macros.h>

#include "ParticleHelper.h"
#include "Particles/ParticleSystem.h"
#include "Particles/ParticleSystemComponent.h"

#include "UltrasonicSensor.generated.h"

UCLASS()
class CARLA_API AUltrasonicSensor : public ASensor
{
    GENERATED_BODY()
    using FUltrasonicData = carla::sensor::data::UltrasonicData;
public:
    AUltrasonicSensor(const FObjectInitializer &ObjectInitializer);
    static FActorDefinition GetSensorDefinition();
    void Set(const FActorDescription &ActorDescription) override;
    
    void SetOwner(AActor *Owner) override;
    
    void Tick(float DeltaSeconds) override;
    
    USceneComponent* Root;
    
    UPROPERTY(VisibleAnywhere)
    TArray<UParticleSystemComponent*>BeamArrayFront;
    UPROPERTY(VisibleAnywhere)
    TArray<UParticleSystemComponent*>BeamArrayBack;
    
private:
    FUltrasonicData m_ultrasonic_data;
    unsigned int m_num_us_front;
    unsigned int m_num_us_back;
    float m_front_fov;
    float m_back_fov;
    float m_max_range;
    std::vector<float> m_angle_offsets_front;
    std::vector<float> m_angle_offsets_back;
    
    UWorld* World;
    FCollisionQueryParams TraceParams;
    FName TraceTag;
    
    std::vector<float> getAngleOffsets(int num_of_rays, float angle);
};
