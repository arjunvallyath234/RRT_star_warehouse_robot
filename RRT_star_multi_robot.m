%% CODE 2 (UPDATED): Dynamic Warehouse (ONLINE RRT* Replanning + Moving Barriers + FIXED OVERLAP + CLEAR TREE + RANDOM ROBOT OBSTACLES)
% FULL CODE MODIFIED:
%   - Replanning ONLY occurs if the path becomes unsafe (collision imminent).
%   - Reactive speed-boost avoidance against moving barriers
%   - If barrier overruns robot, snap back to last safe pose + replan
%   - RANDOM OBSTACLE ROBOTS: user can choose N + each has random color
%   - NO ROBOT TRAJECTORY/TRAIL PLOTTING
%   - OPTION TO SAVE ANIMATION (VideoWriter)

clear; close all; clc;

%% World size
W = 100;
H = 50;
fig = figure('Color','w','Name','Dynamic Warehouse (Event-Based RRT* Replanning)');
ax = axes('Parent',fig); hold(ax,'on'); axis(ax,'equal');
xlim(ax,[0 W]); ylim(ax,[0 H]);
set(ax,'XTick',[],'YTick',[]);
box(ax,'on');

%% ==========================================================
% VIDEO SAVE OPTION (EDIT HERE)
%% ==========================================================
saveVideo   = true;                 % <--- set false to disable saving
videoName   = 'dynamic_warehouse_rrtstar.mp4';  % .mp4 (MPEG-4) recommended
videoQuality = 95;                  % 0..100 (MPEG-4 only)
vw = [];
if saveVideo
    vw = VideoWriter(videoName,'MPEG-4');
    vw.Quality = videoQuality;
end

%% -----------------------------
% 1) Define barrier rectangles
% Each row: [x, y, w, h]
% -----------------------------
barriers0 = [
    10,   2,  2, 40
    22,  8,  2, 35
    38, 10,  2, 34
    52,  0,  2, 35
    65, 20,  2, 12
    65,  0,  2, 12
    65, 40,  2, 12
    75, 22,  2, 14
    75,  0,  2, 12
    85,  6,  2, 38
    95, 12,  2, 20
];

%% ==========================================================
% 2) MOVING BARRIERS OPTIONS (EDIT HERE)
%% ==========================================================
movingBarrierIdx = [2, 4, 5, 8];     % choose barriers by row index
movingYRanges = [
    0,  12;    % barrier 2 (bottom y range)
    0, 12;     % barrier 4
    11, 28;    % barrier 5
    10, 30;    % barrier 8
];
omega = [0.8, 1, 1, 0.8];   % rad/s per moving barrier (or scalar)
phase = [0.8, 1, 1, 0.8];   % rad per moving barrier (or scalar)
yMargin = 0.2;              % margin from world border

%% ==========================================================
% 3) ONLINE NAVIGATION OPTIONS (EDIT HERE)
%% ==========================================================
dt          = 0.05;      % timestep (sec)
simTime     = 50.0;      % total runtime (sec)
robotSpeed  = 6.0;       % base speed (m/s)
goalTol     = 1.0;       % meters
lookAheadT  = 20;        % seconds (check current path soon)

% --- planning robustness horizon (prevents overlap with moving barriers)
useSweptObstaclesForPlanning = true;
planHorizon  = 1.2;       % seconds into the future to "protect" against motion
sweepSamples = 60;        % samples used to bound barrier motion over planHorizon
edgeCheckStep = 0.8;      % collision checking spacing (meters)

% RRT* budget per replan
maxIterPlan   = 550;
stepLen       = 5;
goalRadius    = 4.2;
goalBias      = 0.12;
rewireRadiusMax = 8;
gammaRRT        = 10.0;
fineStepMeters  = 0.35;   % densify final path for smooth robot motion

% ==========================================================
% WALL CLEARANCE OPTION (EDIT HERE)
% ==========================================================
wallClearance = 2;   % meters (set 0 to disable)

%% ==========================================================
% 3.5) REACTIVE COLLISION AVOIDANCE (SPEED BOOST ONLY)
% ==========================================================
useReactiveSpeedBoost = true;
boostDetectRadius    = 8.0;
boostXInflate        = 1.0;
boostYInflate        = 1.0;
maxBoostFactor       = 2.2;
boostGain            = 1.2;
maxRobotSpeedAbs     = 15.0;

%% ==========================================================
% 3.8) RANDOM OBSTACLE ROBOTS OPTIONS (EDIT HERE)
% ==========================================================
% >>> YOUR REQUEST: OPTION TO CHOOSE NUMBER OF OBSTACLE ROBOTS
nRandRobots     = 4;     % <--- CHANGE THIS NUMBER ANYTIME (0,1,2,...)
randRobotSpeed  = 4.5;   % their speed (m/s)
randVelJitter   = 0.0;   % set >0 for slight jitter each step (e.g., 0.2)
randRespawnIfStuck = true;
stuckStepsMax   = round(2.0/dt); % if robot can't move for ~2s, respawn/re-randomize

%% ==========================================================
% 4) VISUAL OPTIONS (EDIT HERE)
%% ==========================================================
showReplanMessage = true;
showRRTNodes      = false;
showRRTTreeEdges  = true;
treeEdgeColor   = [0.90 0.90 0.90];
treeEdgeWidth   = 0.5;

%% -----------------------------
% After dt is known, finalize video writer
% -----------------------------
if saveVideo
    vw.FrameRate = max(1, round(1/dt));
    open(vw);
end

%% -----------------------------
% 5) Draw barriers (rectangle handles)
% -----------------------------
barrierH = gobjects(size(barriers0,1),1);
for i = 1:size(barriers0,1)
    r = barriers0(i,:);
    barrierH(i) = rectangle('Position',r,'FaceColor',[0.20 0.40 0.75], ...
        'EdgeColor',[0.05 0.10 0.25],'LineWidth',1.5);
end

%% -----------------------------
% 6) Robot start + target
% -----------------------------
robotPos  = [3.0, 28.0];
targetPos = [80, 5];
q_goal    = [targetPos(1), targetPos(2)];

drawTarget(ax, targetPos(1), targetPos(2), 1.8);
hStart = plot(robotPos(1), robotPos(2), 'go', 'MarkerFaceColor','g', 'MarkerSize',7);
hGoal  = plot(q_goal(1),   q_goal(2),   'mo', 'MarkerFaceColor','m', 'MarkerSize',7);
title('Dynamic: Moving Barriers + Event-Based RRT* Replanning');

%% -----------------------------
% 7) Create MAIN ROBOT group
% -----------------------------
robotW = 4.5;  robotH = 6.0;
robot  = createRobotGroup(ax, robotW, robotH); % default look
setRobotPose(robot, robotPos(1), robotPos(2));

% NOTE: NO TRAIL PLOT
hPath    = plot(nan,nan,'-','Color',[0.1 0.7 0.1],'LineWidth',3.2);
hPathPts = plot(nan,nan,'o','MarkerFaceColor',[0.1 0.7 0.1], ...
    'MarkerEdgeColor','k','MarkerSize',4);
hNodes = plot(nan,nan,'.','Color',[0.45 0.45 0.45],'MarkerSize',8);
rrtEdgeGroup = hggroup('Parent',ax);

%% ==========================================================
% 7b) Create RANDOM ROBOTS (Obstacle Robots)  [supports N=0]
%% ==========================================================
randRobots = struct('pos', {}, 'vel', {}, 'handle', {}, 'stuckCount', {});
rng(2); % deterministic runs; remove/change seed if you want different each run

for i = 1:nRandRobots
    validStart = false;
    while ~validStart
        rp = [rand()*W, rand()*H];
        if ~isPointInObstacles(rp, barriers0, W, H, wallClearance + 2) && ...
           norm(rp - robotPos) > 10 && norm(rp - q_goal) > 10
            validStart = true;
            randRobots(i).pos = rp;
        end
    end

    ang = rand()*2*pi;
    randRobots(i).vel = [cos(ang), sin(ang)] * randRobotSpeed;

    % random nice-ish color
    c = 0.15 + 0.75*rand(1,3);

    randRobots(i).handle = createRobotGroup(ax, robotW, robotH, c);
    setRobotPose(randRobots(i).handle, randRobots(i).pos(1), randRobots(i).pos(2));
    randRobots(i).stuckCount = 0;
end

%% ==========================================================
% 8) Sanitize moving ranges & omega/phase lengths
%% ==========================================================
nMov = numel(movingBarrierIdx);
if size(movingYRanges,1) ~= nMov || size(movingYRanges,2) ~= 2
    error('movingYRanges must be N x 2, where N=numel(movingBarrierIdx).');
end
if isscalar(omega), omega = repmat(omega,1,nMov); end
if isscalar(phase), phase = repmat(phase,1,nMov); end
if numel(omega) ~= nMov || numel(phase) ~= nMov
    error('omega/phase must be scalar or length equal to movingBarrierIdx.');
end

for m = 1:nMov
    ii = movingBarrierIdx(m);
    hB = barriers0(ii,4);
    yAllowedMin = 0 + yMargin;
    yAllowedMax = (H - hB) - yMargin;

    yMin = movingYRanges(m,1);
    yMax = movingYRanges(m,2);
    if yMin > yMax, tmp=yMin; yMin=yMax; yMax=tmp; end

    if yAllowedMax < yAllowedMin
        yMin = barriers0(ii,2); yMax = barriers0(ii,2);
    else
        yMin = min(max(yMin, yAllowedMin), yAllowedMax);
        yMax = min(max(yMax, yAllowedMin), yAllowedMax);
        if yMax < yMin, yMax = yMin; end
    end
    movingYRanges(m,:) = [yMin yMax];
end

%% ==========================================================
% 9) MAIN LOOP
%% ==========================================================
barriers = barriers0;
prevBarriers = barriers0;
t = 0;

finePath = robotPos;
pathPtr  = 1;

lastSafeRobotPos = robotPos;

% Capture first frame
drawnow;
if saveVideo
    writeVideo(vw, getframe(fig));
end

while t <= simTime
    % --- Update moving barriers ---
    prevBarriers = barriers;
    barriers = updateMovingBarriers(barriers0, barriers, movingBarrierIdx, movingYRanges, omega, phase, t);

    for ii = 1:size(barriers,1)
        set(barrierH(ii),'Position',barriers(ii,:));
    end

    % --- Update Random Robots (Move & Bounce) ---
    currentRobotBarriers = []; % [x, y, w, h] for random robots
    for i = 1:nRandRobots
        p = randRobots(i).pos;
        v = randRobots(i).vel;

        if randVelJitter > 0
            v = v + randVelJitter * randomUnit2(); %#ok<AGROW>
            sp = norm(v);
            if sp > 1e-9
                v = v * (randRobotSpeed/sp);
            else
                v = randomUnit2()*randRobotSpeed;
            end
        end

        pNext = p + v * dt;

        % If next collides (walls/barriers), pick a NEW random direction and don't move this frame
        if isPointInObstacles(pNext, barriers, W, H, wallClearance)
            ang = rand()*2*pi;
            randRobots(i).vel = [cos(ang), sin(ang)] * randRobotSpeed;
            pNext = p;
            randRobots(i).stuckCount = randRobots(i).stuckCount + 1;
        else
            randRobots(i).vel = v;
            randRobots(i).stuckCount = 0;
        end

        % If "stuck" too long, respawn somewhere free
        if randRespawnIfStuck && randRobots(i).stuckCount >= stuckStepsMax
            validStart = false;
            for kk = 1:2000
                rp = [rand()*W, rand()*H];
                if ~isPointInObstacles(rp, barriers, W, H, wallClearance+1) && ...
                   norm(rp - robotPos) > 10 && norm(rp - q_goal) > 10
                    validStart = true;
                    randRobots(i).pos = rp;
                    break;
                end
            end
            ang = rand()*2*pi;
            randRobots(i).vel = [cos(ang), sin(ang)] * randRobotSpeed;
            randRobots(i).stuckCount = 0;
            pNext = randRobots(i).pos;
        end

        randRobots(i).pos = pNext;
        setRobotPose(randRobots(i).handle, pNext(1), pNext(2));

        rectRob = [pNext(1)-robotW/2, pNext(2)-robotH/2, robotW, robotH];
        currentRobotBarriers = [currentRobotBarriers; rectRob]; %#ok<AGROW>
    end

    % Combine obstacles for collision & planning checks
    allObstaclesNow = [barriers; currentRobotBarriers];

    % --- If barrier/obstacle robot overran the robot, snap back ---
    if isPointInObstacles(robotPos, allObstaclesNow, W, H, wallClearance)
        robotPos = lastSafeRobotPos;
        pathPtr  = 1;
        finePath = robotPos;
        setRobotPose(robot, robotPos(1), robotPos(2));
        needPlan = true;
    else
        needPlan = false;
    end

    % --- Goal reached? ---
    if norm(robotPos - q_goal) <= goalTol
        break;
    end

    % --- REPLAN ONLY IF PATH UNSAFE (with random robots included) ---
    if ~needPlan
        needPlan = ~isPathSafeLookahead(robotPos, finePath, pathPtr, allObstaclesNow, ...
            edgeCheckStep, robotSpeed, dt, lookAheadT, W, H, wallClearance);
    end

    if needPlan
        % Planning obstacles:
        if useSweptObstaclesForPlanning
            barriersPlan = sweptObstaclesOverHorizon(barriers0, barriers, movingBarrierIdx, movingYRanges, omega, phase, t, planHorizon, sweepSamples, H, yMargin);
        else
            barriersPlan = barriers;
        end

        % Add random robots as static obstacles (current locations)
        barriersPlan = [barriersPlan; currentRobotBarriers];

        % Clear old tree edges
        if showRRTTreeEdges
            delete(allchild(rrtEdgeGroup));
        end
        if showRRTNodes
            set(hNodes,'XData',nan,'YData',nan);
        end

        params.maxIter = maxIterPlan;
        params.stepLen = stepLen;
        params.goalRadius = goalRadius;
        params.goalBias = goalBias;
        params.edgeCheckStep = edgeCheckStep;
        params.rewireRadiusMax = rewireRadiusMax;
        params.gammaRRT = gammaRRT;

        [pathPoints, dbg] = rrtStarPlan(robotPos, q_goal, barriersPlan, W, H, params, wallClearance);

        % EXTRA SAFETY: ensure collision-free w.r.t current (instantaneous) obstacles
        if ~isempty(pathPoints)
            if ~isPolylineCollisionFree(pathPoints, allObstaclesNow, edgeCheckStep, W, H, wallClearance)
                pathPoints = [];
            end
        end

        if isempty(pathPoints)
            if showReplanMessage
                title(sprintf('Replan @ t=%.2f: FAILED (robot waiting)', t));
            end
        else
            finePath = densifyPath(pathPoints, fineStepMeters);
            pathPtr  = 1;
            set(hPath,'XData',finePath(:,1),'YData',finePath(:,2));
            set(hPathPts,'XData',pathPoints(:,1),'YData',pathPoints(:,2));

            if showReplanMessage
                title(sprintf('Replan @ t=%.2f: OK (nodes=%d)', t, dbg.nNodes));
            end

            if showRRTNodes
                set(hNodes,'XData',dbg.nodes(:,1),'YData',dbg.nodes(:,2));
            end
            if showRRTTreeEdges
                for e = 2:dbg.nNodes
                    p = dbg.nodes(dbg.parent(e),:);
                    q = dbg.nodes(e,:);
                    line([p(1) q(1)], [p(2) q(2)], ...
                        'Color',treeEdgeColor,'LineWidth',treeEdgeWidth, ...
                        'Parent',rrtEdgeGroup,'HandleVisibility','off');
                end
            end

            uistack(hPath,'top');
            uistack(hPathPts,'top');
            uistack(get(robot.T,'Children'),'top');
        end
    end

    % --- Reactive speed boost against moving barriers ---
    vEff = robotSpeed;
    if useReactiveSpeedBoost
        [boostFactor, isThreat] = computeSpeedBoost(robotPos, barriers, prevBarriers, movingBarrierIdx, dt, robotW, ...
            boostDetectRadius, boostXInflate, boostYInflate, boostGain);
        if isThreat
            vEff = robotSpeed * min(maxBoostFactor, boostFactor);
            vEff = min(vEff, maxRobotSpeedAbs);
        end
    end

    % --- Move robot along current plan ---
    stepDist = vEff * dt;
    [robotPosNext, pathPtrNext] = moveAlongPath(robotPos, finePath, pathPtr, stepDist);

    % Hard safety: if next step collides with CURRENT obstacles, don't move
    if isPointInObstacles(robotPosNext, allObstaclesNow, W, H, wallClearance) || ...
       ~isCollisionFree(robotPos, robotPosNext, allObstaclesNow, edgeCheckStep, W, H, wallClearance)
        robotPosNext = robotPos;
        pathPtrNext  = pathPtr;
    else
        lastSafeRobotPos = robotPosNext;
    end

    robotPos = robotPosNext;
    pathPtr  = pathPtrNext;
    setRobotPose(robot, robotPos(1), robotPos(2));

    % Keep visible on top
    uistack(hPath,'top');
    uistack(hPathPts,'top');
    uistack(get(robot.T,'Children'),'top');
    for i = 1:nRandRobots
        uistack(get(randRobots(i).handle.T,'Children'),'top');
    end

    drawnow;

    % ---- SAVE FRAME ----
    if saveVideo
        writeVideo(vw, getframe(fig));
    end

    pause(dt);
    t = t + dt;
end

plot(q_goal(1), q_goal(2), 'p', 'MarkerSize',14, ...
     'MarkerFaceColor',[0.95 0.2 0.2], 'MarkerEdgeColor','k');

legend([hStart hGoal hPath], ...
    {'Start','Goal','Current planned path'}, ...
    'Location','southoutside','Orientation','horizontal');

title(sprintf('Done (t=%.2f). Robot reached goal (or time ended).', t));

drawnow;
if saveVideo
    writeVideo(vw, getframe(fig)); % final frame
    close(vw);
    disp(['Saved video: ', videoName]);
end


%% ==========================================================
% ======================= FUNCTIONS ==========================
%% ==========================================================

function barriers = updateMovingBarriers(barriers0, ~, movingIdx, yRanges, omega, phase, t)
    barriers = barriers0;
    for m = 1:numel(movingIdx)
        ii = movingIdx(m);
        yMin = yRanges(m,1);
        yMax = yRanges(m,2);

        yMid = 0.5*(yMin+yMax);
        A    = 0.5*(yMax-yMin);

        yNew = yMid + A*sin(omega(m)*t + phase(m));
        barriers(ii,2) = yNew;
    end
end

function [boostFactor, isThreat] = computeSpeedBoost(robotPos, barriers, prevBarriers, movingIdx, dt, robotW, detectR, xInfl, yInfl, gain)
    boostFactor = 1.0;
    isThreat = false;

    rx = robotPos(1); ry = robotPos(2);

    for k = 1:numel(movingIdx)
        ii = movingIdx(k);

        rNow  = barriers(ii,:);
        rPrev = prevBarriers(ii,:);

        vy = (rNow(2) - rPrev(2)) / max(dt,1e-9);

        x1 = rNow(1) - xInfl;
        x2 = rNow(1) + rNow(3) + xInfl;
        y1 = rNow(2) - yInfl;
        y2 = rNow(2) + rNow(4) + yInfl;

        if rx < (x1 - robotW/2) || rx > (x2 + robotW/2)
            continue;
        end

        dx = max([x1 - rx, 0, rx - x2]);
        dy = max([y1 - ry, 0, ry - y2]);
        d  = hypot(dx, dy);

        if d > detectR
            continue;
        end

        yC = rNow(2) + 0.5*rNow(4);
        movingToward = ((ry - yC) * vy) < 0;
        if ~movingToward
            continue;
        end

        isThreat = true;

        s = max(0, (detectR - d) / max(detectR,1e-9));
        cand = 1.0 + gain * s/(max(1e-6, (1.0 - s)));
        boostFactor = max(boostFactor, cand);
    end
end

function barriersPlan = sweptObstaclesOverHorizon(barriers0, barriersNow, movingIdx, yRanges, omega, phase, t0, horizon, nSamp, H, yMargin)
    barriersPlan = barriersNow;
    if horizon <= 0 || nSamp < 2
        return;
    end

    ts = linspace(t0, t0+horizon, nSamp);

    for m = 1:numel(movingIdx)
        ii = movingIdx(m);
        r0 = barriers0(ii,:);
        hB = r0(4);

        yMinCmd = yRanges(m,1);
        yMaxCmd = yRanges(m,2);

        yMid = 0.5*(yMinCmd+yMaxCmd);
        A    = 0.5*(yMaxCmd-yMinCmd);

        ys = yMid + A*sin(omega(m)*ts + phase(m));

        yAllowedMin = 0 + yMargin;
        yAllowedMax = (H - hB) - yMargin;
        ys = min(max(ys, yAllowedMin), yAllowedMax);

        yLo = min(ys);
        yHi = max(ys);

        rInfl = r0;
        rInfl(2) = yLo;
        rInfl(4) = hB + (yHi - yLo);

        if rInfl(2) < 0, rInfl(2) = 0; end
        if rInfl(2) + rInfl(4) > H, rInfl(4) = H - rInfl(2); end

        barriersPlan(ii,:) = rInfl;
    end
end

function ok = isPolylineCollisionFree(pathPoints, barriers, stepCheck, W, H, wallC)
    ok = true;
    for i = 1:size(pathPoints,1)-1
        if ~isCollisionFree(pathPoints(i,:), pathPoints(i+1,:), barriers, stepCheck, W, H, wallC)
            ok = false; return;
        end
    end
end

function ok = isPathSafeLookahead(robotPos, finePath, pathPtr, barriers, stepCheck, v, dt, lookAheadT, W, H, wallC)
    ok = true;
    if isempty(finePath) || size(finePath,1) < 2
        ok = false; return;
    end

    steps = max(1, ceil(lookAheadT / dt));
    pos = robotPos;
    ptr = pathPtr;

    for k = 1:steps
        [posNext, ptrNext] = moveAlongPath(pos, finePath, ptr, v*dt);

        if ~isCollisionFree(pos, posNext, barriers, stepCheck, W, H, wallC) || ...
           isPointInObstacles(posNext, barriers, W, H, wallC)
            ok = false; return;
        end
        pos = posNext; ptr = ptrNext;
    end
end

function [posNew, ptrNew] = moveAlongPath(pos, path, ptr, stepDist)
    posNew = pos;
    ptrNew = min(max(ptr,1), size(path,1));

    if isempty(path) || size(path,1) < 2
        return;
    end

    remaining = stepDist;
    while remaining > 1e-9
        if ptrNew >= size(path,1)
            break;
        end
        target = path(ptrNew+1,:);
        v = target - posNew;
        d = norm(v);

        if d < 1e-9
            ptrNew = ptrNew + 1;
            continue;
        end

        if d <= remaining
            posNew = target;
            remaining = remaining - d;
            ptrNew = ptrNew + 1;
        else
            posNew = posNew + (remaining/d)*v;
            remaining = 0;
        end
    end
end

function fine = densifyPath(pathPoints, stepMeters)
    if isempty(pathPoints) || size(pathPoints,1) < 2
        fine = pathPoints; return;
    end
    fine = [];
    for i = 1:size(pathPoints,1)-1
        p1 = pathPoints(i,:);
        p2 = pathPoints(i+1,:);
        L = norm(p2 - p1);
        n = max(2, ceil(L / stepMeters));

        t = linspace(0,1,n)';
        seg = p1 + t.*(p2 - p1);

        if i > 1
            seg = seg(2:end,:);
        end
        fine = [fine; seg]; %#ok<AGROW>
    end
end

function [pathPoints, dbg] = rrtStarPlan(q_start, q_goal, barriers, W, H, params, wallC)
    maxIter         = params.maxIter;
    stepLen         = params.stepLen;
    goalRadius      = params.goalRadius;
    goalBias        = params.goalBias;
    edgeCheckStep   = params.edgeCheckStep;
    rewireRadiusMax = params.rewireRadiusMax;
    gammaRRT        = params.gammaRRT;

    if isPointInObstacles(q_start, barriers, W, H, wallC) || isPointInObstacles(q_goal, barriers, W, H, wallC)
        pathPoints = [];
        dbg.nodes = []; dbg.parent = []; dbg.nNodes = 0;
        return;
    end

    nodes  = q_start;
    parent = 0;
    cost   = 0;

    bestGoalIdx  = 0;
    bestGoalCost = inf;

    for it = 1:maxIter
        if rand < goalBias
            q_rand = q_goal;
        else
            q_rand = [wallC + rand()*(W-2*wallC), wallC + rand()*(H-2*wallC)];
        end

        if isPointInObstacles(q_rand, barriers, W, H, wallC)
            continue;
        end

        [idxNear, ~] = nearestNode(nodes, q_rand);
        q_near = nodes(idxNear,:);

        q_new = steer(q_near, q_rand, stepLen);
        q_new(1) = min(max(q_new(1), wallC), W - wallC);
        q_new(2) = min(max(q_new(2), wallC), H - wallC);

        if isPointInObstacles(q_new, barriers, W, H, wallC)
            continue;
        end

        if ~isCollisionFree(q_near, q_new, barriers, edgeCheckStep, W, H, wallC)
            continue;
        end

        n = size(nodes,1);
        r = min(rewireRadiusMax, gammaRRT * sqrt(log(max(n,2))/max(n,2)));

        nearIdx = find(vecnorm(nodes - q_new, 2, 2) <= r);
        if isempty(nearIdx), nearIdx = idxNear; end

        bestParent  = idxNear;
        bestNewCost = cost(idxNear) + norm(q_new - q_near);

        for k = 1:numel(nearIdx)
            iNear = nearIdx(k);
            qn = nodes(iNear,:);
            cand = cost(iNear) + norm(q_new - qn);
            if cand < bestNewCost
                if isCollisionFree(qn, q_new, barriers, edgeCheckStep, W, H, wallC)
                    bestNewCost = cand;
                    bestParent = iNear;
                end
            end
        end

        nodes  = [nodes; q_new]; %#ok<AGROW>
        parent = [parent; bestParent]; %#ok<AGROW>
        cost   = [cost; bestNewCost]; %#ok<AGROW>
        newIdx = size(nodes,1);

        for k = 1:numel(nearIdx)
            iNear = nearIdx(k);
            if iNear == 1 || iNear == bestParent
                continue;
            end

            qn = nodes(iNear,:);
            newCostViaNew = cost(newIdx) + norm(qn - q_new);

            if newCostViaNew + 1e-9 < cost(iNear)
                if isCollisionFree(q_new, qn, barriers, edgeCheckStep, W, H, wallC)
                    parent(iNear) = newIdx;
                    cost(iNear)   = newCostViaNew;
                end
            end
        end

        if norm(q_new - q_goal) <= goalRadius
            if isCollisionFree(q_new, q_goal, barriers, edgeCheckStep, W, H, wallC)
                goalCostCand = cost(newIdx) + norm(q_goal - q_new);
                if goalCostCand < bestGoalCost
                    bestGoalCost = goalCostCand;
                    if bestGoalIdx == 0
                        nodes  = [nodes; q_goal]; %#ok<AGROW>
                        parent = [parent; newIdx]; %#ok<AGROW>
                        cost   = [cost; bestGoalCost]; %#ok<AGROW>
                        bestGoalIdx = size(nodes,1);
                    else
                        parent(bestGoalIdx) = newIdx;
                        cost(bestGoalIdx)   = bestGoalCost;
                    end
                end
            end
        end
    end

    dbg.nodes = nodes;
    dbg.parent = parent;
    dbg.nNodes = size(nodes,1);

    if bestGoalIdx == 0
        pathPoints = [];
        return;
    end

    idxPath = backtrackPath(parent, bestGoalIdx);
    pathPoints = nodes(idxPath,:);
end

function [idx, dmin] = nearestNode(nodes, q)
    dif = nodes - q;
    ds  = sum(dif.^2, 2);
    [dmin, idx] = min(ds);
    dmin = sqrt(dmin);
end

function qNew = steer(qFrom, qTo, stepLen)
    v = qTo - qFrom;
    d = norm(v);
    if d < 1e-9
        qNew = qFrom;
        return;
    end
    if d <= stepLen
        qNew = qTo;
    else
        qNew = qFrom + (stepLen/d)*v;
    end
end

function idxPath = backtrackPath(parent, goalIdx)
    idxPath = goalIdx;
    cur = goalIdx;
    while parent(cur) ~= 0
        cur = parent(cur);
        idxPath = [cur; idxPath]; %#ok<AGROW>
    end
end

function robot = createRobotGroup(ax, w, h, bodyFaceColor)
    if nargin < 4
        bodyFaceColor = [0.85 0.88 0.92];
    end

    T = hgtransform('Parent', ax);
    x0 = -w/2;  y0 = -h/2;

    bodyW = 0.70*w; bodyH = 0.55*h;
    headW = 0.56*w; headH = 0.22*h;

    bodyX = x0 + 0.15*w;  bodyY = y0 + 0.20*h;
    headX = x0 + 0.22*w;  headY = y0 + 0.75*h;

    rectPatch(T, bodyX, bodyY, bodyW, bodyH, bodyFaceColor, [0.2 0.2 0.2], 1.2);
    rectPatch(T, headX, headY, headW, headH, bodyFaceColor, [0.2 0.2 0.2], 1.2);

    plotLocal(T, -0.12*w, y0 + 0.86*h, 'ko', 4, 'k');
    plotLocal(T, +0.12*w, y0 + 0.86*h, 'ko', 4, 'k');

    lineLocal(T, [-0.18*w, +0.18*w], [y0+0.80*h, y0+0.80*h], [0.2 0.2 0.2], 1.2);

    lineLocal(T, [x0+0.15*w, x0+0.00*w], [y0+0.55*h, y0+0.45*h], [0.2 0.2 0.2], 2);
    lineLocal(T, [x0+0.85*w, x0+1.00*w], [y0+0.55*h, y0+0.45*h], [0.2 0.2 0.2], 2);

    lineLocal(T, [-0.10*w, -0.10*w], [y0+0.20*h, y0+0.05*h], [0.2 0.2 0.2], 2);
    lineLocal(T, [+0.10*w, +0.10*w], [y0+0.20*h, y0+0.05*h], [0.2 0.2 0.2], 2);

    lineLocal(T, [0, 0], [y0+0.97*h, y0+1.05*h], [0.2 0.2 0.2], 1.5);
    plotLocal(T, 0, y0+1.07*h, 'ro', 4, 'r');

    boxCx = -1.5; boxCy = 0; boxW = 2.2; boxH = 2.2;
    drawBoxLocal(T, boxCx, boxCy, boxW, boxH);

    robot.T = T;
end

function setRobotPose(robot, x, y)
    robot.T.Matrix = makehgtform('translate', [x, y, 0]);
end

function drawTarget(ax, cx, cy, r)
    th = linspace(0,2*pi,200);
    plot(ax, cx + (r*1.0)*cos(th), cy + (r*1.0)*sin(th), 'r', 'LineWidth',2);
    plot(ax, cx + (r*0.65)*cos(th), cy + (r*0.65)*sin(th), 'r', 'LineWidth',2);
    plot(ax, cx, cy, 'ro', 'MarkerFaceColor','r', 'MarkerSize',6);
end

function drawBoxLocal(parentT, cx, cy, w, h)
    x = cx - w/2; y = cy - h/2;
    rectPatch(parentT, x, y, w, h, [0.90 0.65 0.25], [0.45 0.25 0.05], 1.0);
    lineLocal(parentT, [x+w*0.15, x+w*0.85], [y+h*0.60, y+h*0.60], [0.55 0.35 0.10], 1.2);
    lineLocal(parentT, [x+w*0.50, x+w*0.50], [y+h*0.35, y+h*0.85], [0.55 0.35 0.10], 1.0);
    rectPatch(parentT, x+w*0.10, y+h*0.10, w*0.25, h*0.25, [0.98 0.82 0.50], 'none', 0.5);
end

function rectPatch(parentT, x, y, w, h, faceCol, edgeCol, lw)
    X = [x, x+w, x+w, x];
    Y = [y, y,   y+h, y+h];
    patch('XData',X,'YData',Y,'FaceColor',faceCol,'EdgeColor',edgeCol, ...
          'LineWidth',lw,'Parent',parentT);
end

function lineLocal(parentT, xs, ys, col, lw)
    line(xs, ys, 'Color', col, 'LineWidth', lw, 'Parent', parentT);
end

function plotLocal(parentT, x, y, style, msz, mfc)
    h = plot(x, y, style, 'MarkerSize', msz, 'Parent', parentT);
    if contains(style,'o') || contains(style,'s') || contains(style,'^') || contains(style,'d') || contains(style,'p')
        set(h, 'MarkerFaceColor', mfc);
    end
end

function inside = isPointInObstacles(p, barriers, W, H, wallC)
    x = p(1); y = p(2);

    if wallC > 0
        if x <= wallC || x >= (W - wallC) || y <= wallC || y >= (H - wallC)
            inside = true;
            return;
        end
    end

    inside = false;
    for i = 1:size(barriers,1)
        r = barriers(i,:);
        if x >= r(1) && x <= (r(1)+r(3)) && ...
           y >= r(2) && y <= (r(2)+r(4))
            inside = true;
            return;
        end
    end
end

function free = isCollisionFree(p1, p2, barriers, stepSize, W, H, wallC)
    d = norm(p2 - p1);
    nSteps = max(2, ceil(d / stepSize));
    tt = linspace(0, 1, nSteps);

    free = true;
    for k = 1:nSteps
        p = p1 + tt(k)*(p2 - p1);
        if isPointInObstacles(p, barriers, W, H, wallC)
            free = false;
            return;
        end
    end
end

function v = randomUnit2()
    a = 2*pi*rand();
    v = [cos(a) sin(a)];
end
