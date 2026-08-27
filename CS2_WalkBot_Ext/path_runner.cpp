#include "path_runner.h"
#include "entities.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <queue>

namespace
{
    constexpr float kMinControlDt = 0.002f;
    constexpr float kMaxControlDt = 0.050f;

    float Clamp01(float value)
    {
        return (std::max)(0.0f, (std::min)(1.0f, value));
    }

    float ClampDt(float dt)
    {
        return (std::max)(kMinControlDt, (std::min)(kMaxControlDt, dt));
    }

    float PlanarLength(const Vector& value)
    {
        return std::sqrt(value.m_flX * value.m_flX + value.m_flY * value.m_flY);
    }

    float ComputeForwardSpeed2D(const Vector& velocity, const QAngle& viewAngles)
    {
        const float yawRad = viewAngles.m_flYaw * (3.14159265358979323846f / 180.0f);
        const float forwardX = std::cos(yawRad);
        const float forwardY = std::sin(yawRad);
        return velocity.m_flX * forwardX + velocity.m_flY * forwardY;
    }

    void UpdateMovementVelocityEstimate(
        PathRunnerState_t& pathRunnerState,
        const Vector& currentPlayerPos,
        bool hasCurrentPlayerVelocity,
        const Vector& currentPlayerVelocity,
        float dt)
    {
        if (!currentPlayerPos.IsValid())
            return;

        if (!pathRunnerState.hasLastObservedPosition)
        {
            pathRunnerState.lastObservedPosition = currentPlayerPos;
            pathRunnerState.hasLastObservedPosition = true;
            pathRunnerState.estimatedVelocity = {};
            return;
        }

        const float safeDt = ClampDt(dt);
        const Vector frameVelocity = (currentPlayerPos - pathRunnerState.lastObservedPosition) / safeDt;
        pathRunnerState.lastObservedPosition = currentPlayerPos;

        Vector sampledVelocity = frameVelocity;
        if (hasCurrentPlayerVelocity && currentPlayerVelocity.IsValid())
            sampledVelocity = currentPlayerVelocity;

        const float smoothing = Clamp01(pathRunnerState.movementVelocitySmoothing);
        const Vector prevEstimatedVelocity = pathRunnerState.estimatedVelocity;
        pathRunnerState.estimatedVelocity = pathRunnerState.estimatedVelocity +
            (sampledVelocity - pathRunnerState.estimatedVelocity) * smoothing;

        const Vector sampledAcceleration = (pathRunnerState.estimatedVelocity - prevEstimatedVelocity) / safeDt;
        pathRunnerState.estimatedAcceleration = pathRunnerState.estimatedAcceleration +
            (sampledAcceleration - pathRunnerState.estimatedAcceleration) * smoothing;
    }

    Vector ComputeCompensatedTargetPosition(
        const PathRunnerState_t& pathRunnerState,
        const Vector& currentPlayerPos,
        const Waypoint& waypoint)
    {
        Vector compensated = waypoint.pos;
        if (!pathRunnerState.enableMovementPrediction || !pathRunnerState.hasLastObservedPosition)
            return compensated;

        const float distanceToWaypoint = currentPlayerPos.Distance(waypoint.pos);
        const float lookAhead = (std::max)(0.0f, (std::min)(0.40f, pathRunnerState.movementPredictionLookAheadSeconds));
        const float gain = (std::max)(0.0f, pathRunnerState.movementPredictionGain);
        const float accelGain = (std::max)(0.0f, pathRunnerState.movementPredictionAccelerationGain);
        const float maxLead = (std::max)(0.0f, pathRunnerState.movementPredictionMaxLead);

        const float speed2D = PlanarLength(pathRunnerState.estimatedVelocity);
        const float speedFactor = Clamp01(speed2D / 260.0f);
        const float nearSuppress = Clamp01((distanceToWaypoint - 6.0f) / 30.0f);
        const float effectiveLookAhead = lookAhead * (0.35f + 0.65f * speedFactor) * nearSuppress;

        Vector lead = pathRunnerState.estimatedVelocity * (effectiveLookAhead * gain);
        lead += pathRunnerState.estimatedAcceleration *
            (0.5f * effectiveLookAhead * effectiveLookAhead * accelGain);
        lead.m_flZ = 0.0f;

        const float leadLength = PlanarLength(lead);
        if (leadLength > maxLead && leadLength > 0.001f)
            lead *= maxLead / leadLength;

        compensated += lead;
        return compensated;
    }

    float PlanarDistance(const Vector& a, const Vector& b)
    {
        const float dx = a.m_flX - b.m_flX;
        const float dy = a.m_flY - b.m_flY;
        return std::sqrt(dx * dx + dy * dy);
    }

    struct DijkstraNode_t
    {
        float distance = 0.0f;
        int index = -1;
    };

    struct DijkstraNodeGreater_t
    {
        bool operator()(const DijkstraNode_t& lhs, const DijkstraNode_t& rhs) const
        {
            return lhs.distance > rhs.distance;
        }
    };

    bool BuildShortestPathTreeFrom(
        const PathManager& pathManager,
        int startIndex,
        std::vector<float>& outDistances,
        std::vector<int>& outPrevious)
    {
        const auto& waypoints = pathManager.GetWaypoints();
        const int waypointCount = static_cast<int>(waypoints.size());
        if (waypointCount <= 0 || startIndex < 0 || startIndex >= waypointCount)
            return false;

        outDistances.assign(static_cast<size_t>(waypointCount), std::numeric_limits<float>::infinity());
        outPrevious.assign(static_cast<size_t>(waypointCount), -1);

        std::priority_queue<DijkstraNode_t, std::vector<DijkstraNode_t>, DijkstraNodeGreater_t> queue;
        outDistances[static_cast<size_t>(startIndex)] = 0.0f;
        queue.push({ 0.0f, startIndex });

        while (!queue.empty())
        {
            const DijkstraNode_t cur = queue.top();
            queue.pop();

            if (cur.index < 0 || cur.index >= waypointCount)
                continue;

            if (cur.distance > outDistances[static_cast<size_t>(cur.index)])
                continue;

            const std::vector<int> nextIndices = pathManager.GetResolvedNextIndices(static_cast<size_t>(cur.index));
            for (int nextIndex : nextIndices)
            {
                if (nextIndex < 0 || nextIndex >= waypointCount)
                    continue;

                const float edgeCost = PlanarDistance(
                    waypoints[static_cast<size_t>(cur.index)].pos,
                    waypoints[static_cast<size_t>(nextIndex)].pos
                );
                const float candidateDistance = cur.distance + edgeCost;
                if (candidateDistance >= outDistances[static_cast<size_t>(nextIndex)])
                    continue;

                outDistances[static_cast<size_t>(nextIndex)] = candidateDistance;
                outPrevious[static_cast<size_t>(nextIndex)] = cur.index;
                queue.push({ candidateDistance, nextIndex });
            }
        }

        return true;
    }

    int ResolveNearestReachableCandidateByPath(
        const std::vector<int>& candidateIndices,
        const std::vector<float>& distances,
        float& outDistance)
    {
        int bestCandidate = -1;
        outDistance = -1.0f;

        for (int candidateIndex : candidateIndices)
        {
            if (candidateIndex < 0 || candidateIndex >= static_cast<int>(distances.size()))
                continue;

            const float distance = distances[static_cast<size_t>(candidateIndex)];
            if (!std::isfinite(distance))
                continue;

            if (bestCandidate < 0 || distance < outDistance)
            {
                bestCandidate = candidateIndex;
                outDistance = distance;
            }
        }

        return bestCandidate;
    }

    int ResolveNextHopFromPreviousTree(
        int startIndex,
        int targetIndex,
        const std::vector<int>& previous)
    {
        if (startIndex < 0 || targetIndex < 0 ||
            startIndex >= static_cast<int>(previous.size()) ||
            targetIndex >= static_cast<int>(previous.size()))
        {
            return -1;
        }

        if (startIndex == targetIndex)
            return targetIndex;

        int node = targetIndex;
        int parent = previous[static_cast<size_t>(node)];
        if (parent < 0)
            return -1;

        while (parent >= 0 && parent != startIndex)
        {
            node = parent;
            parent = previous[static_cast<size_t>(node)];
        }

        if (parent != startIndex)
            return -1;

        return node;
    }
}


namespace
{
    struct CombatTarget_t
    {
        C_CSPlayerPawn* pawn = nullptr;
        Vector bodyPos{};
        float distance = 0.0f;
        int entryIndex = -1;
        bool valid = false;
    };

    CombatTarget_t FindAutoFightTarget(
        const LocalPlayer_t& localPlayer,
        const Vector& localPos,
        float maxRange,
        bool includeTeammates)
    {
        CombatTarget_t best{};
        if (!localPlayer.m_pPlayerPawn)
            return best;

        const std::uint8_t localTeam = localPlayer.m_pPlayerPawn->GetTeamNum();
        if (localTeam == 0)
            return best;

        float bestDist = maxRange;

        for (const EntityObject_t& entityObject : g_EntityList.GetEntities())
        {
            if (!entityObject.m_pEntity || entityObject.m_Type != EEntityType::ENTITY_PLAYER)
                continue;

            auto* pController = reinterpret_cast<CCSPlayerController*>(entityObject.m_pEntity);
            if (!pController || pController == localPlayer.m_pController)
                continue;

            if (!pController->IsPawnAlive())
                continue;

            C_CSPlayerPawn* pEnemyPawn = pController->GetPawnHandle().Get();
            if (!pEnemyPawn)
                continue;

            if (pEnemyPawn->GetHealth() <= 0)
                continue;

            const std::uint8_t enemyTeam = pEnemyPawn->GetTeamNum();
            if (enemyTeam == 0)
                continue;
            if (!includeTeammates && enemyTeam == localTeam)
                continue;

            const Vector origin = pEnemyPawn->GetAbsOrigin();
            if (!origin.IsValid())
                continue;

            const float dist = localPos.Distance(origin);
            if (dist > maxRange || dist >= bestDist)
                continue;

            bestDist = dist;
            best.pawn = pEnemyPawn;
            best.bodyPos = origin;
            best.distance = dist;
            best.entryIndex = entityObject.m_nEntryIndex;
            best.valid = true;
        }

        return best;
    }
    // Returns true if this frame was fully handled (caller should return).
    static bool ProcessAutoFight(
        PathRunnerState_t& pathRunnerState,
        bool hasCurrentPlayerPos,
        const Vector& currentPlayerPos,
        const QAngle& currentPlayerAngle,
        HWND hGameWindow)
    {
        if (!pathRunnerState.autoFight || !hasCurrentPlayerPos)
        {
            SetAttackKeyState(false, pathRunnerState.isAttackPressed);
            pathRunnerState.autoFightTargetEntryIndex = -1;
            return false;
        }

        if (!hGameWindow || GetForegroundWindow() != hGameWindow)
        {
            SetAttackKeyState(false, pathRunnerState.isAttackPressed);
            pathRunnerState.autoFightTargetEntryIndex = -1;
            std::snprintf(pathRunnerState.status, sizeof(pathRunnerState.status),
                "AUTOFIGHT: focus CS2 window");
            return !pathRunnerState.isRunning; // idle mode: fully handled
        }

        const CombatTarget_t combat = FindAutoFightTarget(
            g_Globals.m_LocalPlayer,
            currentPlayerPos,
            pathRunnerState.autoFightRange,
            pathRunnerState.treatTeammatesAsEnemies);

        if (!combat.valid)
        {
            SetAttackKeyState(false, pathRunnerState.isAttackPressed);
            pathRunnerState.autoFightTargetEntryIndex = -1;
            pathRunnerState.autoFightOnTargetSeconds = 0.0f;
            // smooth aim resets on next acquire via s_smoothTargetId
            if (!pathRunnerState.isRunning)
            {
                std::snprintf(pathRunnerState.status, sizeof(pathRunnerState.status),
                    "AUTOFIGHT (idle): no target in range");
                return true;
            }
            return false; // runner continues path
        }

        // Smooth aim to body, then wait shoot-delay, then fire until dead
        Vector eyePos = currentPlayerPos;
        eyePos.m_flZ += 64.0f;
        Vector aimPos = combat.bodyPos;
        aimPos.m_flZ += pathRunnerState.autoFightBodyHeight;

        QAngle targetAngle = CalculateAngleToTarget(eyePos, aimPos);
        targetAngle.m_flPitch = (std::max)(-89.0f, (std::min)(89.0f, targetAngle.m_flPitch));
        targetAngle.m_flRoll = 0.0f;

        // New target -> reset shoot timer
        if (pathRunnerState.autoFightTargetEntryIndex != combat.entryIndex)
            pathRunnerState.autoFightOnTargetSeconds = 0.0f;

        const float dt = ComputeControlDeltaSeconds(pathRunnerState);

        // Smooth factor: UI 0.05..1.0 (higher = faster). Frame-rate independent.
        float smooth = pathRunnerState.autoFightSmooth;
        if (smooth < 0.01f) smooth = 0.01f;
        if (smooth > 1.0f) smooth = 1.0f;
        const float alpha = 1.0f - std::pow(1.0f - smooth, dt * 60.0f);

        // Internal smoothed aim (toward enemy ONLY — no punch in this state)
        // Prevents RCS feedback loop that causes up/down flicking.
        static QAngle s_smoothAim{};
        static int s_smoothTargetId = -1;
        if (s_smoothTargetId != combat.entryIndex)
        {
            s_smoothAim = currentPlayerAngle;
            s_smoothTargetId = combat.entryIndex;
        }

        const float yawToTarget = std::remainderf(targetAngle.m_flYaw - s_smoothAim.m_flYaw, 360.0f);
        const float pitchToTarget = targetAngle.m_flPitch - s_smoothAim.m_flPitch;
        s_smoothAim.m_flYaw += yawToTarget * alpha;
        s_smoothAim.m_flPitch += pitchToTarget * alpha;
        s_smoothAim.m_flPitch = (std::max)(-89.0f, (std::min)(89.0f, s_smoothAim.m_flPitch));
        s_smoothAim.m_flRoll = 0.0f;

        // CS2 RCS: camera shows viewangles + punch*2.
        // To keep crosshair on target: viewangles = aimAtEnemy - punch*2.
        QAngle outAngle = s_smoothAim;
        int shotsFired = 0;
        if (pathRunnerState.autoFightRcs && g_Globals.m_LocalPlayer.m_pPlayerPawn)
        {
            shotsFired = g_Globals.m_LocalPlayer.m_pPlayerPawn->GetShotsFired();
            if (shotsFired > 1) // first bullet has no meaningful spray pattern
            {
                const QAngle punch = g_Globals.m_LocalPlayer.m_pPlayerPawn->GetAimPunchAngle();
                const float strength = (std::max)(0.0f, (std::min)(1.2f, pathRunnerState.autoFightRcsStrength));
                outAngle.m_flPitch -= punch.m_flPitch * 2.0f * strength;
                outAngle.m_flYaw -= punch.m_flYaw * 2.0f * strength;
                outAngle.m_flPitch = (std::max)(-89.0f, (std::min)(89.0f, outAngle.m_flPitch));
            }
        }

        // For alignment / shoot delay: distance of SMOOTH aim to target (ignore punch)
        float yawDelta = yawToTarget;
        float pitchDelta = pitchToTarget;

        bool wroteAngles = false;
        if (g_Globals.m_Offsets.m_uCSGOInput)
        {
            const std::uintptr_t uCSGOInput =
                g_Memory.ReadMemory<std::uintptr_t>(g_Globals.m_Offsets.m_uCSGOInput);
            if (uCSGOInput)
            {
                constexpr std::ptrdiff_t kViewAngleOffset = 0x688;
                g_Memory.WriteMemory<QAngle>(uCSGOInput + kViewAngleOffset, outAngle);
                wroteAngles = true;
            }
        }

        if (!wroteAngles)
        {
            const float pitchScale = (std::max)(0.001f, pathRunnerState.mousePitch * pathRunnerState.mouseSensitivity);
            const float yawScale = (std::max)(0.001f, pathRunnerState.mouseYaw * pathRunnerState.mouseSensitivity);
            float rawX = (yawDelta * alpha) / yawScale;
            float rawY = (pitchDelta * alpha) / pitchScale;
            constexpr float kMaxStep = 35.0f;
            rawX = (std::max)(-kMaxStep, (std::min)(kMaxStep, rawX));
            rawY = (std::max)(-kMaxStep, (std::min)(kMaxStep, rawY));
            const short dx = static_cast<short>(std::lround(rawX));
            const short dy = static_cast<short>(std::lround(rawY));
            if (dx != 0 || dy != 0)
                SendRelativeMouseMove(dx, dy);
        }

        if (pathRunnerState.isRunning)
        {
            SetForwardKeyState(false, pathRunnerState.isForwardPressed);
            SetBackwardKeyState(false, pathRunnerState.isBackwardPressed);
            SetWalkKeyState(false, pathRunnerState.isWalkPressed);
        }

        // Remaining error after this frame's smooth step
        const float angErr = std::fabs(yawDelta) * (1.0f - alpha) + std::fabs(pitchDelta) * (1.0f - alpha);
        // Also check absolute remaining to target
        const float remainYaw = std::fabs(yawDelta) * (1.0f - alpha);
        const float remainPitch = std::fabs(pitchDelta) * (1.0f - alpha);
        const float remainErr = remainYaw + remainPitch;

        const bool aligned = remainErr <= pathRunnerState.autoFightShootAngle;
        if (aligned)
            pathRunnerState.autoFightOnTargetSeconds += dt;
        else
            pathRunnerState.autoFightOnTargetSeconds = 0.0f;

        const float shootDelay = (std::max)(0.0f, pathRunnerState.autoFightShootDelay);
        const bool canShoot = aligned && pathRunnerState.autoFightOnTargetSeconds >= shootDelay;
        SetAttackKeyState(canShoot, pathRunnerState.isAttackPressed);

        pathRunnerState.autoFightTargetEntryIndex = combat.entryIndex;
        std::snprintf(
            pathRunnerState.status,
            sizeof(pathRunnerState.status),
            "AUTOFIGHT%s: #%d dist %.0f err %.1f %s%s",
            pathRunnerState.isRunning ? "" : " (idle)",
            combat.entryIndex,
            combat.distance,
            remainErr,
            canShoot ? "SHOOT" : (aligned ? "WAIT" : "AIM"),
            pathRunnerState.autoFightRcs ? " RCS" : "");

        return true;
    }



} // namespace

void UpdatePathRunner(
    PathRunnerState_t& pathRunnerState,
    PathManager& pathManager,
    bool hasCurrentPlayerPos,
    const Vector& currentPlayerPos,
    bool hasCurrentPlayerVelocity,
    const Vector& currentPlayerVelocity,
    const QAngle& currentPlayerAngle,
    const std::vector<int>& enemyCandidateWaypointIndices,
    HWND hGameWindow)
{
    // Idle autofight: aim/shoot without path runner
    if (!pathRunnerState.isRunning)
    {
        SetForwardKeyState(false, pathRunnerState.isForwardPressed);
        SetBackwardKeyState(false, pathRunnerState.isBackwardPressed);
        SetDuckKeyState(false, pathRunnerState.isDuckPressed);
        SetWalkKeyState(false, pathRunnerState.isWalkPressed);

        if (pathRunnerState.autoFight)
            ProcessAutoFight(pathRunnerState, hasCurrentPlayerPos, currentPlayerPos, currentPlayerAngle, hGameWindow);
        else
        {
            SetAttackKeyState(false, pathRunnerState.isAttackPressed);
            pathRunnerState.autoFightTargetEntryIndex = -1;
        }
        return;
    }

    const auto& waypoints = pathManager.GetWaypoints();
    std::vector<float> shortestDistances;
    std::vector<int> shortestPrevious;

    if (!hGameWindow || GetForegroundWindow() != hGameWindow)
    {
        SetForwardKeyState(false, pathRunnerState.isForwardPressed);
        SetBackwardKeyState(false, pathRunnerState.isBackwardPressed);
        SetDuckKeyState(false, pathRunnerState.isDuckPressed);
        SetWalkKeyState(false, pathRunnerState.isWalkPressed);
        SetAttackKeyState(false, pathRunnerState.isAttackPressed);
        pathRunnerState.emergencyBrakeRemainingSeconds = 0.0f;
        std::snprintf(pathRunnerState.status, sizeof(pathRunnerState.status), "Waiting: focus CS2 window");
        return;
    }

    if (!hasCurrentPlayerPos)
    {
        if (pathRunnerState.isRunning)
            StopPathRunner(pathRunnerState);
        SetAttackKeyState(false, pathRunnerState.isAttackPressed);
        std::snprintf(pathRunnerState.status, sizeof(pathRunnerState.status), "Stopped: no player position");
        return;
    }

    if (waypoints.empty())
    {
        StopPathRunner(pathRunnerState);
        std::snprintf(pathRunnerState.status, sizeof(pathRunnerState.status), "Stopped: no path");
        return;
    }

    if (pathRunnerState.currentWaypointIndex < 0 ||
        pathRunnerState.currentWaypointIndex >= static_cast<int>(waypoints.size()))
    {
        pathManager.ResetRuntimeState();
        pathRunnerState.currentWaypointIndex = FindNearestWaypointIndex(pathManager, currentPlayerPos);
        pathRunnerState.pendingInitialDirectAim = pathRunnerState.directAimOnStart;
        pathRunnerState.trackedEnemyWaypointIndex = -1;
        pathRunnerState.trackedEnemyPathDistance = -1.0f;
        pathRunnerState.waitRemainingSeconds = 0.0f;
        pathRunnerState.lastProcessedArrivalWaypoint = -1;
        ResetHumanizedAimState(pathRunnerState);
        ResetMovementPredictionState(pathRunnerState);
        std::snprintf(
            pathRunnerState.status,
            sizeof(pathRunnerState.status),
            "Running: start from waypoint #%d",
            pathRunnerState.currentWaypointIndex
        );
    }

    bool resolvedEndOfPath = false;
    bool waitingAtEnemyNearestWaypoint = false;
    while (pathRunnerState.currentWaypointIndex >= 0 &&
        pathRunnerState.currentWaypointIndex < static_cast<int>(waypoints.size()))
    {
        int chaseTargetIndex = -1;
        int chaseNextHopIndex = -1;
        if (pathRunnerState.trackNearestEnemy && !enemyCandidateWaypointIndices.empty())
        {
            if (BuildShortestPathTreeFrom(pathManager, pathRunnerState.currentWaypointIndex, shortestDistances, shortestPrevious))
            {
                float bestEnemyPathDistance = -1.0f;
                chaseTargetIndex = ResolveNearestReachableCandidateByPath(
                    enemyCandidateWaypointIndices,
                    shortestDistances,
                    bestEnemyPathDistance
                );
                if (chaseTargetIndex >= 0)
                {
                    chaseNextHopIndex = ResolveNextHopFromPreviousTree(
                        pathRunnerState.currentWaypointIndex,
                        chaseTargetIndex,
                        shortestPrevious
                    );
                    pathRunnerState.trackedEnemyWaypointIndex = chaseTargetIndex;
                    pathRunnerState.trackedEnemyPathDistance = bestEnemyPathDistance;
                }
                else
                {
                    pathRunnerState.trackedEnemyWaypointIndex = -1;
                    pathRunnerState.trackedEnemyPathDistance = -1.0f;
                }
            }
        }
        else
        {
            pathRunnerState.trackedEnemyWaypointIndex = -1;
            pathRunnerState.trackedEnemyPathDistance = -1.0f;
        }

        const Waypoint& currentWaypoint = waypoints[static_cast<size_t>(pathRunnerState.currentWaypointIndex)];
        const float distanceToWaypoint = currentPlayerPos.Distance(currentWaypoint.pos);
        const float effectiveReachDistance = currentWaypoint.radius > 0.0f
            ? currentWaypoint.radius
            : pathRunnerState.waypointReachDistance;
        if (distanceToWaypoint > effectiveReachDistance)
            break;

        // Arrival handling: fire one-shot jump and arm wait timer exactly once
        // per visit. lastProcessedArrivalWaypoint is cleared when the active
        // waypoint index changes, so re-entering the same waypoint re-arms.
        const int currentIdxForArrival = pathRunnerState.currentWaypointIndex;
        if (pathRunnerState.lastProcessedArrivalWaypoint != currentIdxForArrival)
        {
            pathRunnerState.lastProcessedArrivalWaypoint = currentIdxForArrival;
            if ((currentWaypoint.flags & WPF_JUMP) != 0u)
                PulseJumpKey();
            if (currentWaypoint.waitTime > 0.0f)
                pathRunnerState.waitRemainingSeconds = currentWaypoint.waitTime;
        }

        if (pathRunnerState.waitRemainingSeconds > 0.0f)
            break;

        int nextWaypointIndex = -1;
        if (chaseNextHopIndex >= 0 && chaseNextHopIndex != pathRunnerState.currentWaypointIndex)
        {
            nextWaypointIndex = chaseNextHopIndex;
        }
        else if (chaseTargetIndex >= 0 && chaseTargetIndex == pathRunnerState.currentWaypointIndex)
        {
            if (pathRunnerState.waitAtEnemyNearestWaypoint)
                waitingAtEnemyNearestWaypoint = true;
            break;
        }
        else
        {
            nextWaypointIndex = pathManager.ResolveNextWaypointIndex(
                static_cast<size_t>(pathRunnerState.currentWaypointIndex)
            );
        }

        if (nextWaypointIndex < 0 || nextWaypointIndex == pathRunnerState.currentWaypointIndex)
        {
            resolvedEndOfPath = true;
            break;
        }

        pathRunnerState.currentWaypointIndex = nextWaypointIndex;
        pathRunnerState.lastProcessedArrivalWaypoint = -1;
    }

    if (resolvedEndOfPath ||
        pathRunnerState.currentWaypointIndex < 0 ||
        pathRunnerState.currentWaypointIndex >= static_cast<int>(waypoints.size()))
    {
        StopPathRunner(pathRunnerState);
        std::snprintf(pathRunnerState.status, sizeof(pathRunnerState.status), "Finished: reached final waypoint");
        return;
    }

    if (pathRunnerState.waitRemainingSeconds > 0.0f &&
        pathRunnerState.currentWaypointIndex >= 0 &&
        pathRunnerState.currentWaypointIndex < static_cast<int>(waypoints.size()))
    {
        const Waypoint& holdWaypoint = waypoints[static_cast<size_t>(pathRunnerState.currentWaypointIndex)];
        const float crouchDelta = (std::max)(0.0f, pathRunnerState.crouchHeightDelta);
        const bool shouldDuck =
            ((holdWaypoint.flags & WPF_CROUCH) != 0u) ||
            (pathRunnerState.autoCrouchOnLowerTarget &&
                holdWaypoint.pos.m_flZ < (currentPlayerPos.m_flZ - crouchDelta));

        SetDuckKeyState(shouldDuck, pathRunnerState.isDuckPressed);
        SetForwardKeyState(false, pathRunnerState.isForwardPressed);
        SetBackwardKeyState(false, pathRunnerState.isBackwardPressed);
        SetWalkKeyState(false, pathRunnerState.isWalkPressed);
        pathRunnerState.emergencyBrakeRemainingSeconds = 0.0f;

        const float dtWait = ComputeControlDeltaSeconds(pathRunnerState);
        pathRunnerState.waitRemainingSeconds =
            (std::max)(0.0f, pathRunnerState.waitRemainingSeconds - dtWait);

        std::snprintf(
            pathRunnerState.status,
            sizeof(pathRunnerState.status),
            "Waiting: pause at waypoint #%d (%.2fs left)",
            pathRunnerState.currentWaypointIndex,
            pathRunnerState.waitRemainingSeconds
        );
        return;
    }

    if (waitingAtEnemyNearestWaypoint)
    {
        const Waypoint& holdWaypoint = waypoints[static_cast<size_t>(pathRunnerState.currentWaypointIndex)];
        const float crouchDelta = (std::max)(0.0f, pathRunnerState.crouchHeightDelta);
        const bool shouldDuck =
            ((holdWaypoint.flags & WPF_CROUCH) != 0u) ||
            (pathRunnerState.autoCrouchOnLowerTarget &&
                holdWaypoint.pos.m_flZ < (currentPlayerPos.m_flZ - crouchDelta));

        SetDuckKeyState(shouldDuck, pathRunnerState.isDuckPressed);
        SetForwardKeyState(false, pathRunnerState.isForwardPressed);
        SetBackwardKeyState(false, pathRunnerState.isBackwardPressed);
        SetWalkKeyState(false, pathRunnerState.isWalkPressed);
        pathRunnerState.emergencyBrakeRemainingSeconds = 0.0f;
        std::snprintf(
            pathRunnerState.status,
            sizeof(pathRunnerState.status),
            "Waiting: reached enemy-nearest waypoint #%d",
            pathRunnerState.currentWaypointIndex
        );
        return;
    }

    const float dt = ComputeControlDeltaSeconds(pathRunnerState);
    UpdateMovementVelocityEstimate(
        pathRunnerState,
        currentPlayerPos,
        hasCurrentPlayerVelocity,
        currentPlayerVelocity,
        dt
    );


    // ===================== AUTOFIGHT =====================
    if (ProcessAutoFight(pathRunnerState, hasCurrentPlayerPos, currentPlayerPos, currentPlayerAngle, hGameWindow))
        return;
    // =================== END AUTOFIGHT ===================

    int activeTargetIndex = pathRunnerState.currentWaypointIndex;
    const Waypoint& currentTargetWaypoint = waypoints[static_cast<size_t>(activeTargetIndex)];
    const float currentTargetDistance = currentPlayerPos.Distance(currentTargetWaypoint.pos);
    const Vector currentTargetAimPos = ComputeCompensatedTargetPosition(pathRunnerState, currentPlayerPos, currentTargetWaypoint);
    const QAngle currentTargetAngle = CalculateAngleToTarget(currentPlayerPos, currentTargetAimPos);
    const float currentTargetYawDelta =
        std::remainderf(currentTargetAngle.m_flYaw - currentPlayerAngle.m_flYaw, 360.0f);
    const float currentTargetYawDiff = std::fabs(currentTargetYawDelta);

    bool skippedToNextByYaw = false;
    bool skippedToNextByAngle = false;
    if (pathRunnerState.skipToNextOnLargeYaw &&
        currentTargetDistance <= (std::max)(0.0f, pathRunnerState.skipTriggerDistance) &&
        std::fabs(currentTargetYawDelta) >= (std::max)(0.0f, pathRunnerState.skipYawThreshold))
    {
        const int nextWaypointIndex = pathManager.PeekNextWaypointIndex(static_cast<size_t>(activeTargetIndex));
        if (nextWaypointIndex >= 0 &&
            nextWaypointIndex != activeTargetIndex &&
            nextWaypointIndex < static_cast<int>(waypoints.size()))
        {
            const Waypoint& nextWaypoint = waypoints[static_cast<size_t>(nextWaypointIndex)];
            const Vector nextAimPos = ComputeCompensatedTargetPosition(pathRunnerState, currentPlayerPos, nextWaypoint);
            const QAngle nextTargetAngle = CalculateAngleToTarget(currentPlayerPos, nextAimPos);
            const float nextPitchDelta =
                std::remainderf(nextTargetAngle.m_flPitch - currentPlayerAngle.m_flPitch, 360.0f);
            const float nextYawDelta =
                std::remainderf(nextTargetAngle.m_flYaw - currentPlayerAngle.m_flYaw, 360.0f);

            const float yawMoveThreshold = (std::max)(0.0f, pathRunnerState.moveYawThreshold);
            const float pitchMoveThreshold = (std::max)(0.0f, pathRunnerState.movePitchThreshold);
            const bool canWalkStraightToNext =
                std::fabs(nextYawDelta) <= yawMoveThreshold &&
                std::fabs(nextPitchDelta) <= pitchMoveThreshold;

            if (canWalkStraightToNext)
            {
                activeTargetIndex = nextWaypointIndex;
                pathRunnerState.currentWaypointIndex = nextWaypointIndex;
                skippedToNextByYaw = true;
            }
        }
    }

    if (!skippedToNextByYaw &&
        pathRunnerState.skipToNextOnAngleDiff &&
        currentTargetYawDiff >= (std::max)(0.0f, pathRunnerState.skipCurrentYawDiffThreshold))
    {
        const int nextWaypointIndex = pathManager.PeekNextWaypointIndex(static_cast<size_t>(activeTargetIndex));
        if (nextWaypointIndex >= 0 &&
            nextWaypointIndex != activeTargetIndex &&
            nextWaypointIndex < static_cast<int>(waypoints.size()))
        {
            const Waypoint& nextWaypoint = waypoints[static_cast<size_t>(nextWaypointIndex)];
            const Vector nextAimPos = ComputeCompensatedTargetPosition(pathRunnerState, currentPlayerPos, nextWaypoint);
            const QAngle nextTargetAngle = CalculateAngleToTarget(currentPlayerPos, nextAimPos);
            const float nextYawDelta =
                std::remainderf(nextTargetAngle.m_flYaw - currentPlayerAngle.m_flYaw, 360.0f);
            const float nextYawDiff = std::fabs(nextYawDelta);

            if (nextYawDiff <= (std::max)(0.0f, pathRunnerState.skipNextYawDiffThreshold))
            {
                activeTargetIndex = nextWaypointIndex;
                pathRunnerState.currentWaypointIndex = nextWaypointIndex;
                skippedToNextByAngle = true;
            }
        }
    }

    const Waypoint& targetWaypoint = waypoints[static_cast<size_t>(activeTargetIndex)];
    const float crouchDelta = (std::max)(0.0f, pathRunnerState.crouchHeightDelta);
    const bool shouldDuckForHeight =
        ((targetWaypoint.flags & WPF_CROUCH) != 0u) ||
        (pathRunnerState.autoCrouchOnLowerTarget &&
            targetWaypoint.pos.m_flZ < (currentPlayerPos.m_flZ - crouchDelta));
    SetDuckKeyState(shouldDuckForHeight, pathRunnerState.isDuckPressed);

    const bool shouldWalkForPacing =
        ((targetWaypoint.flags & WPF_WALK) != 0u) ||
        (targetWaypoint.desiredSpeed > 0.0f && targetWaypoint.desiredSpeed < 0.75f);
    SetWalkKeyState(shouldWalkForPacing, pathRunnerState.isWalkPressed);
    const Vector targetAimPos = ComputeCompensatedTargetPosition(pathRunnerState, currentPlayerPos, targetWaypoint);
    const QAngle targetAngle = CalculateAngleToTarget(currentPlayerPos, targetAimPos);
    const float pitchDelta = std::remainderf(targetAngle.m_flPitch - currentPlayerAngle.m_flPitch, 360.0f);
    const float yawDelta = std::remainderf(targetAngle.m_flYaw - currentPlayerAngle.m_flYaw, 360.0f);
    const float yawDeltaForAim = yawDelta;

    float sensitivity = pathRunnerState.mouseSensitivity;
    if (sensitivity <= 0.0f)
        sensitivity = 1.0f;
    const float speedMul = (std::max)(0.10f, pathRunnerState.aimSpeedMultiplier);

    const float pitchScale = sensitivity * pathRunnerState.mousePitch;
    const float yawScale = sensitivity * pathRunnerState.mouseYaw;
    const bool initialAcquirePhase = pathRunnerState.pendingInitialDirectAim;
    const float initialAcquireBoost = initialAcquirePhase ? 1.45f : 1.0f;
    const float initialAcquireDampingScale = initialAcquirePhase ? 0.82f : 1.0f;
    float pitchDeltaForAim = 0.0f;
    if (pathRunnerState.allowPitchControl)
    {
        pitchDeltaForAim = pitchDelta;
    }
    else if (pathRunnerState.setPitchOnInitialDirectAim)
    {
        if (initialAcquirePhase && !pathRunnerState.hasLockedPitch)
        {
            pathRunnerState.lockedPitchDeg = targetAngle.m_flPitch;
            pathRunnerState.hasLockedPitch = true;
            pitchDeltaForAim = pitchDelta;
        }
        else if (pathRunnerState.hasLockedPitch)
        {
            pitchDeltaForAim = std::remainderf(
                pathRunnerState.lockedPitchDeg - currentPlayerAngle.m_flPitch,
                360.0f
            );
        }

        const float yawGate = (std::max)(0.0f, pathRunnerState.pitchFloatYawGateDeg);
        const float yawAbs = std::fabs(yawDeltaForAim);
        const float safeDt = (std::max)(0.001f, dt);
        const float yawSpeedDegPerSec = yawAbs / safeDt;
        const float yawAmpNorm = (std::max)(0.0f, yawAbs - yawGate) / (24.0f + yawGate);
        const float yawSpeedNorm = yawSpeedDegPerSec / 300.0f;
        const float activityTarget = (std::min)(1.0f, yawAmpNorm * 0.55f + yawSpeedNorm * 0.45f);
        const float activityFollowRate = 1.2f + activityTarget * 8.5f;
        pathRunnerState.pitchFloatActivity +=
            (activityTarget - pathRunnerState.pitchFloatActivity) *
            (std::min)(1.0f, safeDt * activityFollowRate);

        const float floatFreqBase = (std::max)(0.10f, pathRunnerState.pitchFloatFrequency);
        const float floatFreq =
            floatFreqBase * (0.25f + 2.35f * pathRunnerState.pitchFloatActivity);
        pathRunnerState.pitchFloatPhase += safeDt * floatFreq * 6.28318530718f;
        if (pathRunnerState.pitchFloatPhase > 6.28318530718f)
            pathRunnerState.pitchFloatPhase -= 6.28318530718f;

        const float activityCurve = pathRunnerState.pitchFloatActivity * pathRunnerState.pitchFloatActivity;
        const float floatAmplitude =
            (std::max)(0.0f, pathRunnerState.pitchFloatAmplitudeDeg) *
            (0.10f + 1.65f * activityCurve);
        const float pitchFloat = std::sin(pathRunnerState.pitchFloatPhase) * floatAmplitude;
        pitchDeltaForAim += pitchFloat;
    }

    short mouseDeltaX = 0;
    short mouseDeltaY = 0;
    if (pathRunnerState.aimControlMode == PathRunnerState_t::EAimControlMode::LegacySmooth)
    {
        const float rawMouseDeltaX = pitchScale != 0.0f ? (pitchDeltaForAim / pitchScale) : 0.0f;
        const float rawMouseDeltaY = yawScale != 0.0f ? (-yawDeltaForAim / yawScale) : 0.0f;
        const float acquirePitchSmooth = initialAcquirePhase
            ? pathRunnerState.simplePitchSmooth * 0.42f
            : pathRunnerState.simplePitchSmooth;
        const float acquireYawSmooth = initialAcquirePhase
            ? pathRunnerState.simpleYawSmooth * 0.42f
            : pathRunnerState.simpleYawSmooth;

        mouseDeltaX = ComputeLegacySmoothedMouseDelta(
            rawMouseDeltaX,
            acquirePitchSmooth,
            false
        );
        mouseDeltaY = ComputeLegacySmoothedMouseDelta(
            rawMouseDeltaY,
            acquireYawSmooth,
            false
        );
    }
    else if (pathRunnerState.aimControlMode == PathRunnerState_t::EAimControlMode::WindMouse)
    {
        const float rawMouseDeltaX = pitchScale != 0.0f ? (pitchDeltaForAim / pitchScale) : 0.0f;
        const float rawMouseDeltaY = yawScale != 0.0f ? (-yawDeltaForAim / yawScale) : 0.0f;
        const float windMaxStep = (std::max)(80.0f, pathRunnerState.windMaxStep * speedMul * initialAcquireBoost);
        const float windGravity = (std::max)(1.0f, pathRunnerState.windGravity * speedMul);
        mouseDeltaX = ComputeMouseDeltaWindMouse(
            rawMouseDeltaX,
            dt,
            windGravity,
            pathRunnerState.windForce,
            windMaxStep,
            pathRunnerState.windDamping,
            pathRunnerState.windFittsA,
            pathRunnerState.windFittsB,
            pathRunnerState.windFittsTolerance,
            pathRunnerState.pitchAxis,
            false
        );
        mouseDeltaY = ComputeMouseDeltaWindMouse(
            rawMouseDeltaY,
            dt,
            windGravity,
            pathRunnerState.windForce,
            windMaxStep,
            pathRunnerState.windDamping,
            pathRunnerState.windFittsA,
            pathRunnerState.windFittsB,
            pathRunnerState.windFittsTolerance,
            pathRunnerState.yawAxis,
            false
        );
    }
    else
    {
        const float rawMouseDeltaX = pitchScale != 0.0f ? (pitchDeltaForAim / pitchScale) : 0.0f;
        const float rawMouseDeltaY = yawScale != 0.0f ? (-yawDeltaForAim / yawScale) : 0.0f;
        const float servoMaxSpeed = (std::max)(80.0f, pathRunnerState.servoMaxSpeed * speedMul * initialAcquireBoost);
        const float servoMaxAcceleration =
            (std::max)(500.0f, pathRunnerState.servoMaxAcceleration * speedMul * speedMul * initialAcquireBoost);
        const float servoMaxJerk =
            (std::max)(800.0f, pathRunnerState.servoMaxJerk * speedMul * speedMul * initialAcquireBoost);
        const float acquirePitchResponse = pathRunnerState.servoPitchResponse * initialAcquireBoost;
        const float acquireYawResponse = pathRunnerState.servoYawResponse * initialAcquireBoost;
        const float acquirePitchDamping = pathRunnerState.servoPitchDamping * initialAcquireDampingScale;
        const float acquireYawDamping = pathRunnerState.servoYawDamping * initialAcquireDampingScale;

        mouseDeltaX = ComputeMouseDeltaAdaptiveServo(
            rawMouseDeltaX,
            dt,
            acquirePitchResponse,
            acquirePitchDamping,
            pathRunnerState.servoDeadzone,
            pathRunnerState.servoErrorCurve,
            servoMaxSpeed,
            servoMaxAcceleration,
            servoMaxJerk,
            pathRunnerState.pitchAxis,
            false
        );
        mouseDeltaY = ComputeMouseDeltaAdaptiveServo(
            rawMouseDeltaY,
            dt,
            acquireYawResponse,
            acquireYawDamping,
            pathRunnerState.servoDeadzone,
            pathRunnerState.servoErrorCurve,
            servoMaxSpeed,
            servoMaxAcceleration,
            servoMaxJerk,
            pathRunnerState.yawAxis,
            false
        );
    }

    SendRelativeMouseMove(mouseDeltaY, mouseDeltaX);
    pathRunnerState.pendingInitialDirectAim = false;

    const float yawMoveThreshold = (std::max)(0.0f, pathRunnerState.moveYawThreshold);
    const float pitchMoveThreshold = (std::max)(0.0f, pathRunnerState.movePitchThreshold);
    const bool shouldMoveForward =
        std::fabs(yawDeltaForAim) <= yawMoveThreshold &&
        (!pathRunnerState.allowPitchControl || std::fabs(pitchDelta) <= pitchMoveThreshold);

    if (shouldMoveForward)
    {
        pathRunnerState.emergencyBrakeRemainingSeconds = 0.0f;
        SetBackwardKeyState(false, pathRunnerState.isBackwardPressed);
        SetForwardKeyState(true, pathRunnerState.isForwardPressed);
    }
    else
    {
        const bool wasForwardPressed = pathRunnerState.isForwardPressed;
        SetForwardKeyState(false, pathRunnerState.isForwardPressed);

        bool shouldBrakeBackward = false;
        if (pathRunnerState.emergencyBrakeOnStop)
        {
            if (wasForwardPressed)
            {
                pathRunnerState.emergencyBrakeRemainingSeconds =
                    (std::max)(0.0f, pathRunnerState.emergencyBrakeMaxSeconds);
            }

            const float forwardSpeed = ComputeForwardSpeed2D(pathRunnerState.estimatedVelocity, currentPlayerAngle);
            const float speedThreshold = (std::max)(0.0f, pathRunnerState.emergencyBrakeSpeedThreshold);
            if (pathRunnerState.emergencyBrakeRemainingSeconds > 0.0f &&
                forwardSpeed > speedThreshold)
            {
                shouldBrakeBackward = true;
                pathRunnerState.emergencyBrakeRemainingSeconds =
                    (std::max)(0.0f, pathRunnerState.emergencyBrakeRemainingSeconds - dt);
            }
            else
            {
                pathRunnerState.emergencyBrakeRemainingSeconds = 0.0f;
            }
        }
        else
        {
            pathRunnerState.emergencyBrakeRemainingSeconds = 0.0f;
        }

        SetBackwardKeyState(shouldBrakeBackward, pathRunnerState.isBackwardPressed);
    }

    const float leadDistance = targetAimPos.Distance(targetWaypoint.pos);
    const bool trackingEnemyNow =
        pathRunnerState.trackNearestEnemy &&
        pathRunnerState.trackedEnemyWaypointIndex >= 0 &&
        pathRunnerState.trackedEnemyPathDistance >= 0.0f;
    std::snprintf(
        pathRunnerState.status,
        sizeof(pathRunnerState.status),
        "Running: target #%d dist %.2f yaw %.2f pitch %.2f move %s brake %s lead %.1f%s%s",
        activeTargetIndex,
        currentPlayerPos.Distance(targetWaypoint.pos),
        yawDelta,
        pitchDelta,
        shouldMoveForward ? "ON" : "OFF",
        pathRunnerState.isBackwardPressed ? "ON" : "OFF",
        leadDistance,
        skippedToNextByYaw ? " [SKIP]" :
        (skippedToNextByAngle ? " [ANGLE-SKIP]" : ""),
        trackingEnemyNow ? " [TRACK ENEMY]" : ""
    );
}
