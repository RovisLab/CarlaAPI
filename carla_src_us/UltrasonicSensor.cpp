#include "Carla.h"
#include "Carla/Sensor/UltrasonicSensor.h"

#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Util/BoundingBoxCalculator.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Components/LineBatchComponent.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"

AUltrasonicSensor::AUltrasonicSensor(const FObjectInitializer &ObjectInitializer) 
    : Super(ObjectInitializer)
{
    PrimaryActorTick.bCanEverTick = true;
    TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
    TraceParams.bTraceComplex = true;
    TraceParams.bReturnPhysicalMaterial = false;
    TraceTag = FName("MyTraceTag");
    TraceParams.TraceTag = TraceTag;
    Root = CreateDefaultSubobject<USceneComponent>(FName("Root"));
    static ConstructorHelpers::FObjectFinder<UParticleSystem> ParticleAsset(TEXT("/Game/Carla/Static/Laser/PS_Laser.PS_Laser"));
    
    m_num_us_front = 49;
    m_num_us_back = 49;
    for (size_t i = 0; i < m_num_us_front; ++i)
    {
        UParticleSystemComponent* laser_f = CreateDefaultSubobject<UParticleSystemComponent>(FName("laserf_%d", i));
        laser_f->SetTemplate(ParticleAsset.Object);
        
        BeamArrayFront.Add(laser_f);
    }
     UE_LOG(LogTemp, Warning, TEXT("LOG2"));
    for (size_t i = 0; i < m_num_us_back; ++i)
    {
        UParticleSystemComponent* laser_b = CreateDefaultSubobject<UParticleSystemComponent>(FName("laserb_%d", i));
        laser_b->SetTemplate(ParticleAsset.Object);
        BeamArrayBack.Add(laser_b);
    }
}

FActorDefinition AUltrasonicSensor::GetSensorDefinition()
{
    auto Definition = UActorBlueprintFunctionLibrary::MakeGenericSensorDefinition(
        TEXT("other"),
        TEXT("ultrasonic_sensor"));
    
    FActorVariation num_us_front;
    num_us_front.Id = TEXT("num_us_front");
    num_us_front.Type = EActorAttributeType::Int;
    num_us_front.RecommendedValues = { TEXT("49") };
    num_us_front.bRestrictToRecommended = false;
    
    FActorVariation num_us_back;
    num_us_back.Id = TEXT("num_us_back");
    num_us_back.Type = EActorAttributeType::Int;
    num_us_back.RecommendedValues = { TEXT("49") };
    num_us_back.bRestrictToRecommended = false;
    
    FActorVariation front_us_fov;
    front_us_fov.Id = TEXT("front_us_fov");
    front_us_fov.Type = EActorAttributeType::Float;
    front_us_fov.RecommendedValues = { TEXT("2.8") };
    front_us_fov.bRestrictToRecommended = false;
    
    FActorVariation back_us_fov;
    back_us_fov.Id = TEXT("back_us_fov");
    back_us_fov.Type = EActorAttributeType::Float;
    back_us_fov.RecommendedValues = { TEXT("2.8") };
    back_us_fov.bRestrictToRecommended = false;
    
    FActorVariation max_range;
    max_range.Id = TEXT("max_range");
    max_range.Type = EActorAttributeType::Float;
    max_range.RecommendedValues = { TEXT("31.0") };
    max_range.bRestrictToRecommended = false;
    
    Definition.Variations.Append({num_us_front, num_us_back, front_us_fov, back_us_fov, max_range});
    
    return Definition;
}

void AUltrasonicSensor::Set(const FActorDescription &Description)
{
    Super::Set(Description);
    
    m_num_us_front = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt(
        "num_us_front",
        Description.Variations,
        49);
    m_num_us_back = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt(
        "num_us_back",
        Description.Variations,
        49);
    
    m_front_fov = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
        "front_us_fov",
        Description.Variations,
        2.8f);
    m_back_fov = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
        "back_us_fov",
        Description.Variations,
        2.8f);
    m_max_range = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
        "max_range",
        Description.Variations,
        31.f);
    
    m_angle_offsets_front = getAngleOffsets(m_num_us_front, m_front_fov);
    m_angle_offsets_back = getAngleOffsets(m_num_us_back, m_back_fov);

    World = GetWorld();
    
    
    // World->DebugDrawTraceTag = TraceTag;
}

void AUltrasonicSensor::SetOwner(AActor *Owner)
{
    Super::SetOwner(Owner);
}

std::vector<float> AUltrasonicSensor::getAngleOffsets(int num_of_rays, float angle)
{
    std::vector<float> offsets;
    float step = angle / (num_of_rays - 1);
    
    for (int i = -(num_of_rays - 1) / 2; i < (num_of_rays - 1)/2 + 1; i++)
    {
        offsets.push_back(i * step);
    }

    return offsets;
}

/*
 * Data measurement is performed here
 */
void AUltrasonicSensor::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    
    constexpr float TO_METERS = 1e-2;
    constexpr float TO_CENTIMETERS = 1e2;
    
    /* Raycast and get Ultrasonic values */
    const FTransform& ActorTransform = GetActorTransform();
    const FRotator& ActorRotation = GetActorRotation();
    const FVector& UsLocation = GetActorLocation();
    const FVector& ForwardVector = GetActorForwardVector();
    const float Offset = 2.f * TO_CENTIMETERS;
    
    /*Get front and back US start points*/
    const FVector FrontUsStartPoint = UsLocation + ForwardVector * Offset;
    const FVector BackUsStartPoint = UsLocation - ForwardVector * Offset;
    
    FCriticalSection Mutex;
    GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
    
    m_ultrasonic_data.ClearRange();
    
    FHitResult OutHit(ForceInit);
    
    //Front US sensor raycast    
    for (size_t i = 0; i < m_num_us_front; ++i)
    {
        const float Angle = FMath::DegreesToRadians(ActorRotation.Yaw) + m_angle_offsets_front[i];
        float Sin, Cos;
        FMath::SinCos(&Sin, &Cos, Angle);
        const FVector EndPoint = {
            FrontUsStartPoint.X + m_max_range * TO_CENTIMETERS * Cos,
            FrontUsStartPoint.Y + m_max_range * TO_CENTIMETERS * Sin,
            FrontUsStartPoint.Z
        };
        
        const bool Hit = World->LineTraceSingleByChannel(
            OutHit,
            FrontUsStartPoint, 
            EndPoint, 
            ECC_GameTraceChannel2, 
            TraceParams, 
            FCollisionResponseParams::DefaultResponseParam);

        // BeamArrayFront[i]->SetBeamSourcePoint(0, FrontUsStartPoint, 0);
        Mutex.Lock();
        if (Hit)
        {
            
            // BeamArrayFront[i]->SetBeamTargetPoint(0, OutHit.ImpactPoint, 0);
            // DrawDebugLine(World, FrontUsStartPoint, OutHit.ImpactPoint, FColor::Green, false, 0.05, 0, 0.5);
            // DrawDebugLine(World, OutHit.ImpactPoint, EndPoint, FColor::Red, false, 0.05, 0, 0.5);
            m_ultrasonic_data.WriteRange({OutHit.Distance * TO_METERS, Angle});
        }
        else
        {
            // BeamArrayFront[i]->SetBeamTargetPoint(0, EndPoint, 0);
            // DrawDebugLine(World, FrontUsStartPoint, EndPoint, FColor::Green, false, 0.05, 0, 0.5);
            m_ultrasonic_data.WriteRange({m_max_range, Angle});
        }
        Mutex.Unlock();
    }
    
    //Rear US sensor raycast
    for (size_t i = 0; i < m_num_us_back; ++i)
    {
        const float Angle = FMath::DegreesToRadians(ActorRotation.Yaw) + m_angle_offsets_back[i];
        float Sin, Cos;
        FMath::SinCos(&Sin, &Cos, Angle);
        const FVector EndPoint = {
            BackUsStartPoint.X - m_max_range * TO_CENTIMETERS * Cos,
            BackUsStartPoint.Y - m_max_range * TO_CENTIMETERS * Sin,
            BackUsStartPoint.Z
        };
        
        const bool Hit = World->LineTraceSingleByChannel(
            OutHit,
            BackUsStartPoint, 
            EndPoint, 
            ECC_GameTraceChannel2, 
            TraceParams, 
            FCollisionResponseParams::DefaultResponseParam);
        Mutex.Lock();
        // BeamArrayBack[i]->SetBeamSourcePoint(0, BackUsStartPoint, 0);
        if (Hit)
        {
            // BeamArrayBack[i]->SetBeamTargetPoint(0, OutHit.ImpactPoint, 0);
            // DrawDebugLine(World, BackUsStartPoint, OutHit.ImpactPoint, FColor::Green, false, 0.05, 0, 0.5);
            // DrawDebugLine(World, OutHit.ImpactPoint, EndPoint, FColor::Red, false, 0.05, 0, 0.5);
            m_ultrasonic_data.WriteRange({OutHit.Distance * TO_METERS, Angle});
        }
        else
        {
            // BeamArrayBack[i]->SetBeamTargetPoint(0, EndPoint, 0);
            // DrawDebugLine(World, BackUsStartPoint, EndPoint, FColor::Green, false, 0.05, 0, 0.5);
            m_ultrasonic_data.WriteRange({m_max_range, Angle});
        }
        Mutex.Unlock();
    }
    
    /* Send data */
    auto Stream = GetDataStream(*this);
    GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();
    Stream.Send(*this, m_ultrasonic_data, Stream.PopBufferFromPool());
}

