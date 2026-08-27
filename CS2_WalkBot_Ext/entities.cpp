#include "entities.h"
#include "memory.h"
#include "interfaces.h"
#include <cstring>

// Entity list entry stride. Most builds use 0x70. If enemies stay empty, try 0x78.
static constexpr std::ptrdiff_t kEntityListStride = 0x70;

C_BaseEntity* C_BaseEntity::GetBaseEntity(int nIdx) noexcept
{
    const std::uintptr_t uEntityList = g_Globals.m_uEntityList;
    if (!uEntityList)
        return nullptr;

    const std::uintptr_t uListEntry = g_Memory.ReadMemory<std::uintptr_t>(
        uEntityList + (0x8 * ((nIdx & 0x7FFF) >> 0x9)) + 0x10);
    if (!uListEntry)
        return nullptr;

    return g_Memory.ReadMemory<C_BaseEntity*>(uListEntry + kEntityListStride * (nIdx & 0x1FF));
}

C_BaseEntity* CBaseHandle::Get() const
{
    if (!IsValid())
        return nullptr;

    C_BaseEntity* pEntity = C_BaseEntity::GetBaseEntity(GetEntryIndex());
    if (!pEntity || pEntity->GetRefEHandle() != *this)
        return nullptr;

    return pEntity;
}

static bool IsPlayerControllerName(const std::string& name)
{
    if (name.empty())
        return false;

    // Accept both schema class name and designer name variants
    if (name == "CCSPlayerController" || name == "cs_player_controller")
        return true;

    // Some builds return slightly different strings
    if (name.find("PlayerController") != std::string::npos)
        return true;

    return false;
}

void EntityList::UpdateEntities()
{
    m_vecEntities.clear();
    m_vecEntities.reserve(64);

    // ============================================================
    // Primary method: Index-based iteration (most reliable)
    // Controllers live in low entity indices (usually < 64)
    // ============================================================
    if (g_Globals.m_uEntityList)
    {
        for (int i = 1; i < 64; ++i)
        {
            C_BaseEntity* pBaseEntity = C_BaseEntity::GetBaseEntity(i);
            if (!pBaseEntity)
                continue;

            const std::string schemaName = pBaseEntity->GetSchemaName();
            if (!IsPlayerControllerName(schemaName))
                continue;

            m_vecEntities.push_back(EntityObject_t{
                pBaseEntity,
                i,
                EEntityType::ENTITY_PLAYER
                });
        }
    }

    // ============================================================
    // Fallback: Linked list via GameEntitySystem (if index method found nothing)
    // ============================================================
    if (m_vecEntities.empty())
    {
        CEntityIdentity* pEntityIdentity = g_Globals.m_GameEntitySystem.m_pFirst;
        if (!pEntityIdentity)
            return;

        int safety = 0;
        for (; pEntityIdentity != nullptr && safety < 2048; pEntityIdentity = pEntityIdentity->m_pNext(), ++safety)
        {
            CEntityInstance* pInstance = pEntityIdentity->m_pInstance();
            if (!pInstance)
                continue;

            C_BaseEntity* pBaseEntity = reinterpret_cast<C_BaseEntity*>(pInstance);
            const std::string schemaName = pBaseEntity->GetSchemaName();
            if (!IsPlayerControllerName(schemaName))
                continue;

            m_vecEntities.push_back(EntityObject_t{
                pBaseEntity,
                pBaseEntity->GetRefEHandle().GetEntryIndex(),
                EEntityType::ENTITY_PLAYER
                });
        }
    }
}
