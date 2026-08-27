#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include "math_types.h"
#include "sdk/client_dll.hpp"
#include "memory.h"

#define INVALID_EHANDLE_INDEX 0xFFFFFFFF
#define ENT_ENTRY_MASK 0x7FFF
#define NUM_SERIAL_NUM_SHIFT_BITS 15

// Macro for offset-based member access with ReadMemory
#define OFFSET(TYPE, NAME, OFFSET_VAL) \
[[nodiscard]] __forceinline TYPE NAME() const noexcept { \
    return g_Memory.ReadMemory<TYPE>(reinterpret_cast<std::uintptr_t>(this) + OFFSET_VAL); \
};

// Forward declarations
class CEntityIdentity;
class CEntityInstance;
class CGameSceneNode;
class C_BaseEntity;
class C_CSPlayerPawn;
class CCSPlayerController;

// Entity Handle
class CBaseHandle
{
public:
    CBaseHandle() noexcept :
        m_uIndex(INVALID_EHANDLE_INDEX) {
    }

    CBaseHandle(const std::uint32_t uIndex) noexcept :
        m_uIndex(uIndex) {
    }

    CBaseHandle(const int nEntry, const int nSerial) noexcept
    {
        m_uIndex = nEntry | (nSerial << NUM_SERIAL_NUM_SHIFT_BITS);
    }

    [[nodiscard]] bool IsValid() const
    {
        return m_uIndex != INVALID_EHANDLE_INDEX;
    }

    [[nodiscard]] int GetEntryIndex() const
    {
        return static_cast<int>(m_uIndex & ENT_ENTRY_MASK);
    }

    [[nodiscard]] int GetSerialNumber() const
    {
        return static_cast<int>(m_uIndex >> NUM_SERIAL_NUM_SHIFT_BITS);
    }

    [[nodiscard]] C_BaseEntity* Get() const;

    [[nodiscard]] bool operator==(const CBaseHandle& other) const
    {
        return m_uIndex == other.m_uIndex;
    }

    [[nodiscard]] bool operator!=(const CBaseHandle& other) const
    {
        return !(*this == other);
    }

    std::uint32_t m_uIndex;
};

// Typed handle
template<typename T>
class CHandle : public CBaseHandle
{
public:
    CHandle() = default;
    CHandle(std::uint32_t uIndex) : CBaseHandle(uIndex) {}

    T* Get() const
    {
        return reinterpret_cast<T*>(CBaseHandle::Get());
    }
};

class CEntityIdentity
{
public:
    // Updated 26.08.2026
    OFFSET(std::uint32_t, m_flags, cs2_dumper::schemas::client_dll::CEntityIdentity::m_flags); // 0x30
    OFFSET(CEntityIdentity*, m_pNext, cs2_dumper::schemas::client_dll::CEntityIdentity::m_pNext); // 0x58

    // First pointer in identity is the entity instance
    OFFSET(CEntityInstance*, m_pInstance, 0x0);

    // Designer name (CUtlSymbolLarge) - reliable for class checks
    OFFSET(const char*, m_designerNamePtr, 0x20);

    [[nodiscard]] int GetEntryIndex() const noexcept
    {
        const std::uint32_t uHandle = g_Memory.ReadMemory<std::uint32_t>(reinterpret_cast<std::uintptr_t>(this) + 0x10);
        return static_cast<int>(uHandle & ENT_ENTRY_MASK);
    }

    [[nodiscard]] int GetSerialNumber() const noexcept
    {
        const std::uint32_t uHandle = g_Memory.ReadMemory<std::uint32_t>(reinterpret_cast<std::uintptr_t>(this) + 0x10);
        return static_cast<int>(uHandle >> NUM_SERIAL_NUM_SHIFT_BITS);
    }

    [[nodiscard]] std::string GetDesignerName() const
    {
        // CUtlSymbolLarge is effectively a pointer to the string
        const std::uintptr_t pName = g_Memory.ReadMemory<std::uintptr_t>(reinterpret_cast<std::uintptr_t>(this) + 0x20);
        if (!pName)
            return {};
        return g_Memory.ReadMemoryString(pName);
    }
};

class CEntityInstance
{
public:
    [[nodiscard]] CBaseHandle GetRefEHandle() const
    {
        CEntityIdentity* pIdentity = m_pEntity();
        if (!pIdentity)
            return CBaseHandle();

        return CBaseHandle(pIdentity->GetEntryIndex(), pIdentity->GetSerialNumber() - (pIdentity->m_flags() & 1));
    }

    [[nodiscard]] std::string GetSchemaName() const
    {
        // Method 1: designerName from identity (most reliable)
        CEntityIdentity* pIdentity = m_pEntity();
        if (pIdentity)
        {
            std::string name = pIdentity->GetDesignerName();
            if (!name.empty())
                return name;
        }

        // Method 2: old schema chain (fallback)
        std::uintptr_t uSchemaNameAddress = g_Memory.ReadMemory(
            reinterpret_cast<std::uintptr_t>(this) + 0x10,
            { 0x8, 0x78, 0x8 }
        );
        if (uSchemaNameAddress == 0U)
            return {};

        return g_Memory.ReadMemoryString(uSchemaNameAddress);
    }

    OFFSET(CEntityIdentity*, m_pEntity, 0x10)
};

class CGameSceneNode
{
public:
    // Updated 26.08.2026
    OFFSET(Vector, GetAbsOrigin, cs2_dumper::schemas::client_dll::CGameSceneNode::m_vecAbsOrigin); // 0xC8
    OFFSET(QAngle, GetAbsRotation, cs2_dumper::schemas::client_dll::CGameSceneNode::m_angAbsRotation);
};

// Base entity
class C_BaseEntity : public CEntityInstance
{
public:
    [[nodiscard]] static C_BaseEntity* GetBaseEntity(int nIdx) noexcept;

    // Updated 26.08.2026
    OFFSET(CGameSceneNode*, GetGameSceneNode, cs2_dumper::schemas::client_dll::C_BaseEntity::m_pGameSceneNode); // 0x330
    OFFSET(std::uint8_t, GetTeamNum, cs2_dumper::schemas::client_dll::C_BaseEntity::m_iTeamNum);                 // 0x3E7
    OFFSET(Vector, GetAbsVelocity, cs2_dumper::schemas::client_dll::C_BaseEntity::m_vecAbsVelocity);
    OFFSET(std::int32_t, GetHealth, cs2_dumper::schemas::client_dll::C_BaseEntity::m_iHealth);
    OFFSET(std::uint8_t, GetLifeState, cs2_dumper::schemas::client_dll::C_BaseEntity::m_lifeState);            // 0x3F8

    [[nodiscard]] Vector GetAbsOrigin() const
    {
        CGameSceneNode* pSceneNode = GetGameSceneNode();
        if (!pSceneNode)
            return {};

        return pSceneNode->GetAbsOrigin();
    }

    [[nodiscard]] QAngle GetAbsRotation() const
    {
        CGameSceneNode* pSceneNode = GetGameSceneNode();
        if (!pSceneNode)
            return {};

        return pSceneNode->GetAbsRotation();
    }
};

// Player pawn
class C_CSPlayerPawn : public C_BaseEntity
{
public:
    // Updated 26.08.2026
    OFFSET(CHandle<CCSPlayerController>, GetOriginalController, cs2_dumper::schemas::client_dll::C_CSPlayerPawnBase::m_hOriginalController); // 0x1478
    OFFSET(std::uintptr_t, GetAimPunchServices, cs2_dumper::schemas::client_dll::C_CSPlayerPawn::m_pAimPunchServices); // 0x14B8
    OFFSET(std::int32_t, GetShotsFired, cs2_dumper::schemas::client_dll::C_CSPlayerPawn::m_iShotsFired); // 0x1C8C

    // Aim punch angle from CCSPlayer_AimPunchServices::m_predictableBaseAngle (0x50)
    [[nodiscard]] QAngle GetAimPunchAngle() const
    {
        const std::uintptr_t services = GetAimPunchServices();
        if (!services)
            return {};
        return g_Memory.ReadMemory<QAngle>(services + cs2_dumper::schemas::client_dll::CCSPlayer_AimPunchServices::m_predictableBaseAngle);
    }
};

// Player controller
class CCSPlayerController : public C_BaseEntity
{
public:
    // Updated 26.08.2026
    OFFSET(CHandle<C_CSPlayerPawn>, GetPawnHandle, cs2_dumper::schemas::client_dll::CBasePlayerController::m_hPawn);
    OFFSET(bool, IsPawnAlive, cs2_dumper::schemas::client_dll::CCSPlayerController::m_bPawnIsAlive); // 0x91C

    CHandle<C_CSPlayerPawn> m_hPawn;
};

enum class EEntityType : std::uint8_t
{
    ENTITY_UNKNOWN = 0,
    ENTITY_PLAYER = 1
};

struct EntityObject_t
{
    C_BaseEntity* m_pEntity = nullptr;
    int m_nEntryIndex = -1;
    EEntityType m_Type = EEntityType::ENTITY_UNKNOWN;
};

class EntityList
{
public:
    void UpdateEntities();
    void Clear()
    {
        m_vecEntities.clear();
    }

    [[nodiscard]] const std::vector<EntityObject_t>& GetEntities() const
    {
        return m_vecEntities;
    }

private:
    std::vector<EntityObject_t> m_vecEntities;
};

inline EntityList g_EntityList;
