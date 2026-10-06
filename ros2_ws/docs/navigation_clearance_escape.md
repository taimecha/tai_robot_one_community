## Local fork-tip raster contact during retreat, 2026-10-02

After several successful live goals, navigation to `(3.561430, 0.114829,
1.557230)` aborted from `(0.106, -0.470)` facing about 39.7 degrees. Route
checks reported blocked initial turns; the controller reported no clear local
tracking arc; Smac Lattice later reported `Start occupied`. The checked
16 cm retreat stopped. A read-only post-goal costmap/TF capture reproduced
the retreat rejection in isolated ROS domain 231. Exact C++ collision
diagnostics placed its first blocked sample 1 cm rearward: global footprint
cost 253, local cost 254. Live `/scan_filtered` points at `(0.639, 0.165)` in map,
within about 1.3 cm of the local lethal cell and about 1.9 cm outside the
padded fork edge at the stopped pose. The laser return is real; the extra
raster cell contact during translation occurs at the front while reversing
away from it. The capture was taken after the abort and cannot reproduce
every earlier moving costmap update.

The local costmap padding is now 25 mm instead of 30 mm; the configured
polygon already includes 20 mm, leaving 45 mm total planning margin around
the measured physical outline. Global costmap padding stays 30 mm. Isolated
replay of the stopped capture passes the complete 16 cm rear guard with only
5 mm reduction; inserting a lethal local rear cell makes the same replay
reject retreat. The live filtered scan had no returns inside the revised
padded footprint at the stopped pose. Nav2 BackUp, live local observations,
full-body route validation and Collision Monitor's FootprintApproach still
check their own corridors. Physical
execution of the new margin requires a Nav2 restart and a new operator goal.

## Terminal pivot from an existing padding contact, 2026-10-02

The latest live goal `(2.364308, -0.354141, -2.915173)` selected a 2.056 m
SE2Arc route. After about eight seconds the route guard canceled FollowPath;
subsequent searches failed and navigation aborted. The stopped robot was
4.223 cm from the requested XY, with 32.44 degrees of heading error. The
post-goal read-only capture found one lethal global raster edge cell
`(1.909, -1.018)` in the padded fork footprint, no core-body contact and no
live local footprint contact. A clockwise correction released that initial
contact immediately and ended clear. This capture reproduces the stopped
geometry, rather than every moving costmap update before cancellation.

Terminal validation now permits a shortest pivot out of an existing contact
confined to the extra 3 cm global padding. Only initially intersected lethal
padding cells are excluded in a private validation copy. The original core
and live full footprint are checked over the entire turn; all new obstacles
and unknown cells remain blocking. The full footprint must end clear on the
original maps, and returning to a contact after leaving it is rejected.
This applies only at an accepted terminal XY on a prepared path or a nearby
Direct proposal, so retrying the same goal can reach final alignment too.
No published costmap, physical footprint or acceptance tolerance is changed.

The captured selected path failed the isolated replay before this fix.
Synthetic coverage includes release from a map-only fork-padding contact,
retrying a Direct goal, and rejection of live, core or newly encountered
contacts. No real robot action or velocity command was sent for verification.
After the fix, the captured prepared path and same-goal Direct proposal both
passed replay, and the complete `test_path_footprint_bt` CTest group passed.
The rebuilt BT library was installed atomically; the running Nav2 process
keeps its previous mapped library until the user restarts navigation.

## Nearby rear goals and stronger advance pockets, 2026-10-02

The failed live goal `(3.125522, -0.077951, 1.563481)` started near
`(-0.446, 1.517)`. Recovery advanced 30 cm to a pocket that admitted a
15-degree turn and a short departure. Native Spin then reported collision.
The robot retreated about 53 cm, the rear-footprint guard stopped BackUp,
subsequent route searches failed, and the goal aborted. A read-only capture
at the stopped pose reproduced the rear rejection; the global costmap had
lethal edge cells in the padded rear footprint 16 cm farther back. The capture
was taken after the goal, so it is not a replay of the earlier moving scene.

Forward recovery now requires the whole goal-facing turn at its proposed
endpoint, both before starting and during advance. All local turn exits
require a checked departure up to 75 cm rather than the former 30 cm.
The rear guard reports the blocked rear corridor explicitly. Obstacles remain
blocking. BackUp cancellation result waiting is 200 ms (was 20 ms); normal
server acknowledgment waiting is 100 ms. The previous wait expired before
the 20 Hz behavior loop had returned its cancellation result.

The existing Direct proposal also offers `DirectReverse` when the requested
goal is within 1 m, its signed longitudinal displacement is behind the robot
by more than 5 cm, and goal heading differs by at most 5 degrees. Goals near
the rear axis get a straight rear route. Offset rear goals get a cubic curve
whose endpoint tangents preserve start and goal chassis headings. Full body
poses, the final correction and fresh local/global costmaps are checked.
Only consistently rearward paths within 1 m endpoint displacement and 1.5 m
route length enable reverse tracking; ambiguous or obstructed curves fail
validation. FollowPath drives these routes at no more than 0.08 m/s, checks
native time-to-collision with the negative velocity, and brakes before a gear
change. Arrival tolerances stay 5 cm and 5 degrees.

All four selected CTest groups passed, including the actual controller with
synthetic TF/costmaps: straight reverse without a half-turn, curved reverse
steering, arrival stop, forward/reverse braking and rear collision rejection.
Additional route tests cover a 1 m rear destination, lateral rear curves,
changed-heading/long/sideways rejection, full-corridor obstacles, and refusing
forward advance when only a small angled pocket fits. These are isolated
regressions; no real goal or velocity command was sent for this revision.

## Pi reset from GitHub, 2026-10-02

Base: GitHub `taimecha/tai_robot_one`, commit `40b48f7d140ed1c294f45e085cb5f07cf3eac0b0`.
The previous local workspace was moved to
`/home/admin/tai_robot_one/tai_robot_one_before_reset_20261002_125616` before
cloning the baseline into `/home/admin/tai_robot_one/tai_robot_one`.

This revision adds continuous retreat and adjusts speeds. Starting BackUp
commits immediately; crossing any rear-distance threshold does not release
that commitment. The same BackUp action stays active until the full checked
exit turn and a corridor up to 75 cm toward the goal fit at the current pose.
The turn has the existing extra footprint/braking margins. The subtree stops
BackUp, waits for braking, rechecks the whole turn and corridor, and then
executes Spin. Rear obstruction and stale sensor/TF data still stop recovery.
A NO_VALID_PATH result permits this local recovery only with local blockage
or an existing recovery/turn-limit state; a distant unreachable goal alone
does not authorize reverse motion.

Cruise translation is 0.28 m/s (was 0.20), heading alignment and recovery Spin
are capped at 0.35 rad/s (was 0.30). Final yaw correction is capped at
0.24 rad/s with a 0.22 rad/s active floor. Arrival speed taper starts within
0.30 m of remaining path (was 0.90 m), with a 0.10 m/s minimum approach speed.
Obstacle, curvature, collision and acceleration limits still regulate speed
when needed. Checked recovery BackUp retains its 0.08 m/s speed.
Final goal acceptance is 0.05 m and 0.0872664626 rad (5 cm and 5 degrees),
shared by the goal checker, controller and route footprint validation.

Regression coverage includes one active BackUp across 0.31, 0.62, 0.93 and
1.24 m, rejecting a pocket with less than the longer exit corridor, rechecking
that corridor before Spin, and the actual retreat subtree's handoff from
BackUp to Spin. These are isolated tests, not real robot motion results.

Fresh builds of `tai_robot_one`, `sllidar_ros2`, `astra_camera_msgs` and
`astra_camera` completed. All three selected CTest groups passed:
`test_stop_envelope`, `test_rotation_sweep`, and `test_path_footprint_bt`.
The optional capture replay was skipped without a supplied capture file.
The Nav2 launch argument check passed using only this newly installed workspace.
Two old route-search test expectations were aligned with the GitHub baseline's
existing policy; no route planner or route-ranking code was changed.

# Real navigation: footprint clearance and continuous escape

`nav_real.launch.py`, `nav_sim.launch.py` and `slam_nav_sim.launch.py` select
the same `navigate_real_clearance_escape.xml` tree in normal mode.
`motion_test:=true` on the real robot keeps its separate diagnostic tree.
The simulation launches route Nav2's `/cmd_vel_safe` output to the simulated
base and disable the `VelocityStop` polygon; `FootprintApproach` and the
controller / BackUp collision checks remain enabled. The real launch keeps
its own collision monitor settings.

For a goal up to 6 m away, the real tree first tries a dense straight route
from the actual pose to the exact requested goal. It includes a checked
stationary turn onto the line when necessary and a checked final goal-yaw turn.
It separately tries a tangent circular arc if the bearing is at most 0.80 rad,
the turn radius is at least 0.90 m and the final heading correction is at most 1 rad.
It checks the entire padded body, launch sweep and shortest terminal sweep
against both costmaps; no initial map overlap is ignored. A blocked or
ineligible line/arc falls through to the existing planners. These paths avoid
cell-grid kinks and start turning while advancing when there is room.
The tree compares the eligible Direct and DirectArc routes with GridShortest,
GridBased, SE2Arc and SE2Fallback before moving. GridShortest uses a smaller
inflation-cost multiplier to offer a shorter route through clear gaps; the
existing GridBased remains available when that route fails the same body check.
SE2Arc supplies continuous forward curves
with a 0.60 m minimum turning radius; SE2Fallback supplies lattice paths with
stationary turn primitives. Every candidate passes the full-body validator.
`RankedRouteSearch` selects the lowest motion-cost valid candidate and retains it while
it remains usable. A reactive fresh-data/footprint guard checks during motion;
a blocked route or geometric controller failure first requests fresh planning
from the actual pose. Recovery also refreshes all candidates after a running
route stops, rather than executing a longer route from an obsolete start pose.
Successful FollowPath completion during recovery is remembered for that exact
goal, so RecoveryNode's return to its primary does not send another FollowPath.
The completion flag is cleared at the next tree execution. The goal checker
is non-stateful: every success requires current map-relative XY and yaw
within 4 cm and 5 degrees. The controller rechecks XY during final yaw
alignment and clears its arrival acceptance if the vehicle drifts out of
tolerance. A checked final pivot finishes its yaw correction before
correcting XY, so small localization or skid drift does not alternate the
heading target every tick. Drift beyond 15 cm rejects the pivot and requests
fresh planning. Terminal turns starting within 1.8 rad of the requested yaw
use the measured reliable 0.20 rad/s rate within 25 cm of the goal; larger
turns retain 0.30 rad/s so they finish before controller progress expires.
After entering arrival, the controller
can correct backward drift of at most 15 cm with a straight
retreat at up to 0.08 m/s when lateral error is below 90% of XY tolerance and
goal-yaw error is at most 0.35 rad. It stops angular motion first and checks
the entire retreat corridor plus native collision projection before moving.
This exception does not enable ordinary reverse path tracking.
The final path pose uses a zero timestamp so goal acceptance follows current
localization, rather than map-to-odom at the original request time; see
[ControllerServer::isGoalReached](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_controller/src/controller_server.cpp).
AMCL's differential motion noise alphas are 0.03. In the isolated Gazebo
terminal-turn test, the previous 0.20 values let map-to-odom jump over 10 cm
while odometry stayed fixed; the lower values completed the same goal inside
4 cm XY / 5 degrees. Real odometry and lidar alignment need a hardware check.
Quantized heading-aware endpoints are replaced with the exact requested pose
before validation. An aligned lattice turn is released within 5 cm after skid
turn drift, avoiding repeated attempts to revisit that already-completed cusp.
At a running stationary-turn cusp, prepared-path validation advances past
the old heading and checks the remaining sweep from the actual current yaw.
It retains the forward prefix until the controller's 1.5 cm turn-entry threshold
and still rejects long-way final alignment. Within the 4 cm XY tolerance,
prepared-path checks validate the actual body and shortest final rotation at
the accepted current XY, matching the non-stateful goal checker. They do not
require another translation to the exact endpoint. This arrival phase requires
at most 15 cm of route remaining: a checked departure from a nearby goal must
finish its route before the controller latches final yaw. Complete checked
route searches remain permitted near the goal; restricted local escape still
requires more than 10 cm of goal distance.
Candidate validation additionally checks goal-bearing and final-yaw turns at
the first XY acceptance point and at 4 cm longitudinal/lateral tracking
offsets when a substantial final turn remains. A nominal path that would trap
the robot after a small tracking error is rejected before motion.
A changed goal cancels the old action and compares routes from the actual pose.
This finds the lowest motion-cost validated candidate within the planning budget;
it does not prove global optimality across all possible continuous trajectories.
The live footprint can arrive up to one controller tick ahead of odometry TF.
When that lead is at most 150 ms, validation uses the latest transform for the
footprint instead of canceling an otherwise clear route for a brief TF gap.

The condition uses both global and local raw costmaps near the robot, and the
global costmap farther away. Samples are at most half a cell / 1 cm in combined
translation and fork-tip angular travel. Occupied edges, enclosed obstacle
cells, unknown cells and footprints outside the map are rejected. Initial /
terminal rotations use only the shortest correction; initial rotations may
try the other complete sweep. Recovery evaluates possible turns rather than intentionally rotating
until contact or an emergency stop.

After failure, recovery compares fresh Direct, DirectArc, GridShortest,
GridBased, SE2Arc and SE2Fallback candidates by motion score again. It next searches complete routes from checked
forward starts before falling back to local manoeuvres. A distant footprint/
terminal failure or occupied goal does NOT authorize unchecked local recovery
motion. Start-occupied, near-body path
collision and failed control/progress can authorize local escape; terminal
alignment inside the 4 cm XY acceptance radius cannot. Within 25 cm of the
goal, routes over 2.5 m are deferred while checked local escape is tried;
they remain available as a last fallback. Error state is reset at the start
of a new tree execution and before the recovery alternatives.
For a goal 5–35 cm behind the robot, checked local escape may start even if
the forward-only planners have no short route. It first requires 25 cm of
measured, collision-checked rear travel before accepting a turn, so a brief
in-place spin does not send the robot away from the nearby target.
Forward-departure proposals are skipped for such a goal, so an apparently
short path starting by driving farther away does not outrank the checked rear
departure. Long forward routes remain a final fallback if no local exit fits.

If the healthy recovery search is exhausted, the outer recovery allows two
additional stopped planning cycles. Each waits 1.5 s for observations,
invalidates the old path and replans from the actual pose. Motion history is
retained, and sensor/TF/controller faults prevent these retries. This does
not clear either costmap or claim exhaustive route search.

## Compare departure manoeuvres before a long detour

`RankedRouteSearch` first collects Direct, DirectArc, GridShortest, GridBased,
SE2Arc and SE2Fallback.
If no current-pose route fits, or the shortest exceeds `1.35 * straight_distance
+ 0.50 m` for a goal farther than 0.05 m, it also compares the complete routes
from forward starts at 0.15, 0.30, ..., 1.20 m before ANY motion. Up to 30
 candidates plus nine goal-aligned approaches share a 20 s planning budget.
When a validated direct line needs a terminal pivot above 0.6 rad, the nine
aligned approaches are compared directly, without unrelated forward-departure
trials. A blocked direct line keeps the existing obstacle routes in the lead.
The checked departure is part of
both route length and footprint validation. Long routes remain eligible if no
shorter departure manoeuvre fits. Ordinary short routes avoid the extra search.
This prevents accepting a long forward-only Dubins loop solely because a
safe short translation was never considered before recovery.

For any candidate with a different travel and goal heading, the validator
checks the entire final rotation at the first accepted XY point and at small
tracking offsets. A safe rotate-in-place finish stays eligible, including on
long routes. When that rotation is blocked, aligned approach candidates
remain available.

When this extended search is needed, it also asks all three native planners
to reach a standoff 0.25, 0.50 or 0.75 m behind the requested goal along its
final heading. The joined route explicitly aligns at that standoff and then
advances straight to the exact goal. The entire connection, alignment sweep
and final corridor must pass the same full-body checks. These candidates
compete by total XY length with the other routes; there are at most 39 trials.
This permits alignment before entering a tight goal region where the final
pose fits but rotating after an angled arrival would collide.

`/navigation/stationary_turn_limited` reports the controller's persistent
stationary-turn guard. `route_control_available` skips futile FollowPath
retries while that guard requires translation; checked local escape remains
available. A measured 4 cm odometry displacement permits retry, matching the
controller guard. An AMCL correction cannot grant that permission.

When the reactive route guard cancels FollowPath first, its action error can
still be zero. The explicit turn-limit signal therefore also authorizes checked
local escape for a nonterminal goal, subject to the existing fault gates. A
limited turn must translate before another Spin: an otherwise usable current
turn no longer suppresses a checked forward pocket or checked rear corridor.
The rear handoff uses the same measured 4 cm odometry threshold, so the latched
controller signal need not wait for another FollowPath to release BackUp.
A changed goal may reach setPlan() to reset the previous goal's controller limit.

Tracking also selects the outgoing tangent only after skipping an aligned
completed cusp. A slight lateral skid offset at that old cusp must not cause
another heading correction toward a point behind or beside the robot.

## Ranked complete forward routes

`ForwardRouteSearch` runs before the old restricted local escape heuristics.
While stopped, it proposes starts at 0.15, 0.30, ..., 1.20 m straight ahead,
bounded by remaining goal distance and the fresh full-body global/local
departure corridor. For each proposal the native ComputePathToPose action
uses the explicit hypothetical start (`use_start=true`) for ALL THREE of
GridBased, SE2Arc and SE2Fallback: up to 24 routes, not the first successful
planner at each start. The full
departure from the actual current pose is prepended at 2.5 cm intervals;
any quantized launch-heading correction is placed AT the future start.
The entire joined route, including every rotation primitive and the exact
requested terminal pose, must pass PathFootprintClear before it is retained.
Local observations cover the whole departure prefix even beyond one metre.
Proposal checks issue no velocity or navigation actions.

Retained candidates are ranked by XY path length plus estimated wheel travel
for any final in-place rotation beyond 0.5 rad (0.45 m per extra radian).
The approach direction is measured about 25 cm before the goal so a final
one-cell grid kink cannot disguise a large pivot.
This makes a short, aligned approach preferable to a route only a few
centimetres shorter that requires a near-180-degree pivot at the goal.
Smaller turns do not change length priority. Numerical score ties (0.1 mm
buckets) use heading variation and stop-to-turn manoeuvres as a tiebreaker.
The ranking includes the full departure prefix. Costmap inflation costs inside
the native planners continue to preserve clearance; this is candidate comparison,
not an exhaustive geometric shortest-path search.
The selected route is revalidated against current pose/maps before execution,
and a reactive route/data guard continues checking while FollowPath runs.
In the shared navigation tree, blockage or control patience/no-progress/no-valid-
control triggers fresh route comparison from the actual pose. Invalid
alternatives are skipped. A running route stays
selected; searching does not restart every controller tick. A changed goal or
halt cancels the current planner/action and clears the search choices.

Only after the finite forward route choices are exhausted does the existing
checked local forward-pocket/turn/continuous-retreat fallback run. Full routes
may be searched after distant path-validation failure because they prove the
entire connection to the goal; distant failure still cannot authorize blind
local escape. Controller unknown/invalid-controller/TF/invalid-path/timeout
faults stop route retries and cannot authorize subsequent local motion even
if the planner reported Start occupied. Stale sensors/maps/TF still stop
movement. A 60 s stopped planning deadline prevents a hung planner waiting
indefinitely; after it, only previously validated and freshly revalidated
candidates may execute. Failure/abort can still occur for unreachable goals,
faults, or exhaustion of this bounded candidate set.

Local escape first checks whether a useful turn is available at the current
pose. Otherwise it searches forward turning/exit pockets at 15 cm intervals,
up to 1.20 m and bounded by remaining goal distance. Clear translation alone
is no longer sufficient: the destination must also permit a checked turn and
30 cm forward exit, or a goal-directed straight corridor up to 75 cm. All
intervening translations, turn sweeps and exits use the unmodified global AND
fresh local maps (including beyond one metre). Both signed turn directions
are evaluated at 15-degree candidates plus the exact goal bearing; heading
alignment is preferred. During the first local attempts an angled detour may end retreat without the
entire goal-facing sweep fitting: its complete selected turn AND a 30 cm
forward exit must fit. A bare partial turn without a checked exit still fails.
After braking, that selected turn executes immediately and the full route
is replanned from the resulting pose; this local exit is not proof of arrival.
Turning-pocket checks add a private 4 cm footprint margin and
0.07 rad braking margin; the published footprint and rear translation
footprint are not altered. Tiny turns still pointing into the dead end are
not considered progress. This is local geometric
escape selection, not proof that an entire route to the destination exists.
DriveOnHeading is reactively guarded over the remaining translation and
destination turning pocket. A lost pocket/corridor, collision result 723, or
successful advance with no remaining exit switches to checked retreat rather
than aborting immediately. A stopped 0.3 s Wait and fresh-data gate separate
the canceled forward action from reverse, so opposing behavior commands do
not overlap during direction changes. Drive/Spin timeout, TF, invalid-input and unknown
faults still propagate instead of authorizing movement after a system fault.
Spin COLLISION_AHEAD (703) instead rechecks the fresh full-body 16 cm rear
corridor. If safe, the SAME navigation goal continues with continuous retreat;
if blocked or stale it stops safely. Before another recovery turn it requires
another freshly checked useful exit with margin. The failed turn heading is
excluded within 15 cm of its recorded odometry launch location, but another
safe heading can execute immediately, without an arbitrary extra retreat.
If a Spin collision has no recorded attempted heading, the conservative
15 cm measured retreat is still required. A geometrically failed forward
escape immediately commits to seeking different space, rather than choosing
another tiny partial turn in the same pocket (see region memory below).
Turning in place, advancing, or an AMCL correction does not count as reverse
travel. Ordinary replanning resets do not erase this commitment; a new
navigation execution/goal does.

## Region memory and lost turning pockets

Local manoeuvres do not prove a complete connection to the goal. Record
executed/attempted local turns and completed local advances in a goal-scoped
odometry region. After two such adjustments within 45 cm, stop proposing
another short local advance or partial turn there. A geometrically stopped
advance commits immediately. Ordinary ResetEscapeState, path invalidation
and a successful partial Spin preserve this memory; a new goal/execution
clears it. Complete normal Direct/GridBased/SE2 routes remain eligible and
fully collision-checked; merely seeing a planner path flash is not evidence
of an executable body-safe route.

During commitment, a retreat can still end immediately if the entire selected
goal-facing turn, private margins and its forward departure corridor fit at
the CURRENT pose. A 15-degree partial turn alone cannot interrupt it. A
successful goal-facing Spin releases commitment; otherwise 30 cm signed
odometry retreat with at least 25 cm net displacement releases the regional
restriction so alternatives can be reconsidered. These are hysteresis
thresholds, not blind distances: rear obstruction, native collision checks
or stale data stop reverse first. Pure forward/sideways travel and rotation
do not satisfy the reverse threshold. This finite heuristic is not an
exhaustive search or a guarantee that no physical route exists after abort.

If a turning pocket disappears during the stopped braking handoff, or the
selected Spin fails its fresh geometric launch check, `lost_exit` allows one
fresh planning cycle and further checked retreat instead of immediate abort.
It requires an actual pending handoff/geometric launch failure, error code
zero, fresh data and a full-body safe rear corridor. It is consumed once.
Spin timeout, TF, invalid-input and unknown faults cannot use this allowance;
BackUp faults without a turning handoff cannot use it either.

Only with no forward/turn exit is BackUp allowed. It stays active at 0.08 m/s,
not restarted every 5 cm. A reactive guard checks clearance at 5 Hz and halts
BackUp immediately when a useful exit exists at the CURRENT pose, or the full-body
16 cm rear corridor is obstructed. All original behavior-server collision
checks, velocity smoothing and Collision Monitor remain downstream. The
10 m / 180 s action limits are fault bounds, not a blind requested retreat;
the guard normally cancels much earlier. Missing/stale sensor/TF data or a
non-geometric motion-action fault terminate recovery safely. Rear blockage
stops reverse motion, but after measured retreat the tree first requests one
new planning cycle from that stopping pose. Arrival at an
unreachable goal is not guaranteed, and a stalled motor is not geometric proof
that no manoeuvre exists.

While reversing, an open short forward segment or a hypothetical pocket ahead
does NOT interrupt the action and send the vehicle back into the old dead end.
The same BackUp goal remains active while retreating toward a turning pocket.
After a usable exit is reached, BackUp is halted, a 0.3 s stopped Wait permits
braking, fresh data and the turn are rechecked, and Spin executes before
replanning. A failed Spin geometry check commits another measured retreat;
other faults stop safely. Executed turn headings are remembered in odom within
15 cm of their launch location, so the same local turn is not retried endlessly;
translation or a new goal permits reconsideration. DriveOnHeading
has a 25 s fault deadline to accommodate the longer checked forward search.
Simulation, footprint dimensions and the downstream safety chain are unchanged.

A retreat can reach a pose from which a compound SE2 route is feasible even
though the stricter local goal-facing turning-pocket heuristic still rejects
it. `replan_after_retreat` prevents treating that rejection as proof that no
route exists. When the reactive guard geometrically stops a retreat, or
BackUp reports COLLISION_AHEAD (714), at least 5 cm signed reverse displacement
in odom permits exactly one extra planning cycle. The previous path is
invalidated, a 0.3 s stopped Wait allows braking/new observations, and fresh
data are required again. Normal Direct/GridBased/SE2 planning and recovery
then run from the new actual pose, including checked forward pockets if the
new route still does not fit. Each complete candidate route still passes
the full published-footprint validator before FollowPath can execute it.
This does NOT relax the private turning-pocket margin or accept an unchecked
partial turn: local angled exits must include a checked forward corridor,
and every subsequent complete route still requires full validation.
No periodic restart is added to the running BackUp action. The replan allowance
is consumed once, is cleared for a new goal/execution, and cannot be earned
by standing still, moving forward or an AMCL pose jump. BackUp timeout, TF,
invalid input and unknown faults never authorize it; stale data stop it too.

## Narrow padding-only retreat release

A global-only overlap confined to the extra 3 cm padding no longer necessarily
locks reverse recovery. A private validation map can omit initial lethal cells
in that padding only if the unmodified core body and the entire live local body
are clear. Unknown cells, core collisions, newly encountered global obstacles
and any local observation remain blocking. The whole 30 cm rear release must
fit and its endpoint must fit the unmodified global/local footprint. Normal
rear guarding still checks 16 cm continuously; no published map, physical
footprint, native BackUp collision check or Collision Monitor is disabled.
If the static map overlaps the core, navigation still stops; this is not a
general waiver for Start occupied or evidence that an obstacle is fictitious.

SafeRotationRPP supports the opposite complete sweep away from the goal and a 378-degree
stationary-turn budget. A measured 4 cm translation clears that budget, including
after it was tripped. Final XY acceptance is latched while aligning heading;
final turn tolerance is taken from the goal checker instead of hardcoded 2.6
degrees. Final alignment recomputes the signed shortest angle every cycle and
brakes before reversing angular direction. Goal acceptance is 4 cm / 5 degrees.
Within 50 cm of the endpoint, alignment never selects a long opposite sweep;
a blocked short turn requires another checked manoeuvre instead of a full lap.

Tracking uses the immediate path segment for initial heading rather than the
far lookahead carrot. A lattice rotate-in-place primitive limits lookahead:
the vehicle advances to that position before turning. Translation retains RPP
speed / cost regulation and collision checks, and tries shorter carrots when
the longer tracking arc would collide. This avoids rejecting a planned
"advance 15 cm, then turn" route by trying to turn at the original position.

GridShortest and GridBased XY path orientations are reconstructed from segment
tangents and the requested terminal goal pose before validation AND controller
execution. Initial alignment remains subject to the same footprint sweep.
RPP searches only the first 0.30 m of remaining path when pruning. The default
2.50 m search can skip to a later grid segment near the start of a winding
route, causing repeated in-place turns while the robot has not traveled.
For a plan start within 25 cm of the robot, RPP aligns with its outgoing
tangent. This avoids turning toward a stale start cell after localization
shifts the robot a few centimetres during an in-place pivot.
The terminal XY is also restored to the actual requested goal, not its
discretized map-cell corner; this exact endpoint must still pass collision checks.
Very short Smac2D plans can otherwise contain identity intermediate yaw and
ask a west-facing robot to spin almost 180 degrees at the destination.
Lattice SE2 orientations and rotation primitives are never rewritten.
Raw Jazzy Smac2D intermediate cell corners are converted to physical cell
centres once. A retained prepared route is never shifted again. This removes
a half-cell start bias that can make a short straight route request a large
initial turn while the geometry validator assumes a nearly straight departure.
When within 10 cm of the route, controller heading alignment uses its first
travel-segment tangent rather than a biased tiny cell-to-robot bearing.
An upcoming in-place turn still limits lookahead and is not anticipated early;
large off-route offsets retain point-bearing alignment.

The BT condition composes current odom->base with the latest map->odom.
It requires fresh odometry, costmaps and footprint; the timestamp of a held
stationary AMCL transform is not incorrectly treated as stale odometry.
Private condition nodes ignore process-wide ROS node-name remaps.
All conditions now share one costmap/footprint subscription cache per tree.
An asynchronous readiness step waits up to two seconds for initial callbacks
and matching TF, with no motion command, instead of aborting a new goal on its
first tick. Once motion has begun, stale data still stops recovery immediately.

Forward departure may ignore an existing global raster overlap confined to
the extra 3 cm padding when the complete live local footprint is clear.
This affects only a private validation copy: core-body cells, unknown cells,
observed local obstacles and newly encountered map obstacles remain blocked.
It handles a stationary vehicle touching a map cell at the padded rear edge
without disabling obstacle layers or altering the published footprint.

A separate explicitly checked local escape can also depart a map-only overlap
of the starting body: only the initially intersected lethal map cells are
masked in a private scratch copy for that forward translation; unknown and
newly intersected cells remain blocked. Every sampled local footprint must
fit, and the endpoint must fit the unmodified global map. Normal path
validation and reverse/turn sweeps do NOT ignore core-body map obstacles.
This handles Start occupied without deleting map data or inventing a new pose.
It does not establish that localization is correct or that LiDAR sees low /
occluded obstacles; the operator must verify those physical limitations.

Tests run in isolated ROS domain 231, never issue velocity commands, and cover
clear and narrow corridors, an obstacle enclosed inside the fork footprint,
stale maps, held localization with fresh odometry, advance-before-turn,
short final-angle correction, interpolated motion and turn-budget recovery.
The real tree is loaded against installed Nav2 port declarations with motion
actions replaced by test builders. Loaded-floor behaviour and obstacle / fork
clearance still require observation on the actual robot after restart.
Regression tests also cover rejecting recovery for distant obstacles,
preferring a viable turn to reverse, retaining one continuous backup action
and halting it when an exit appears, and clearing old goal error codes.
Additional regression scenarios include a clear short forward dead end with
a rear turning pocket (one continuous retreat until the pocket), a forward
pocket beyond an arbitrary 30 cm step, and a newly blocked rear corridor.
Spin-collision fallback tests cover continuous additional retreat rather than
immediate abort/repeating the same turn, checking the whole SELECTED sweep and
forward exit rather than requiring the goal-facing sweep, accepting a different
checked heading without arbitrary extra retreat, and refusing reverse after
timeout/TF faults. The actual autoremap retreat subtree is executed with mock
native actions: one BackUp is halted and one Spin executes as soon as an angled
pocket becomes available. Other tests cover a lost forward corridor switching
to continuous BackUp, failed-advance fault rejection/reset, and narrowly
allowing map-only padding release while rejecting core/local/unknown contacts
or a new rear obstacle.
Retreat-stop tests additionally verify one halted continuous BackUp followed
by a new planning attempt, consuming the allowance once, invalidating the
old route, permitting geometric collision replanning, and rejecting timeout/
TF faults, no displacement, forward travel, localization jumps and old goals.

`scripts/inspect_navigation_clearance` captures current costmaps / footprint /
TF and requests paths from hypothetical starts, without sending navigation
or velocity actions. Captures can be replayed in the isolated BT tests with
`TAI_CLEARANCE_CAPTURE=/absolute/path/to/capture.json`. The diagnostic probes
use an explicitly supplied goal yaw; RViz screenshot goal yaw is not assumed.
`TAI_CLEARANCE_ADVANCE=-0.20` can replay a hypothetical rearward start from
the capture (and its matching SE2 probe) without moving the real vehicle.
`TAI_CLEARANCE_MOTION=turn` / `reverse_needed` select local escape queries;
`TAI_CLEARANCE_EXPECT_FAILURE=1` selects a deliberately blocked expectation.
`TAI_CLEARANCE_MOTION=replan_after_retreat` replays a synthetic 15 cm retreat
to the capture's stopping pose, checks that the rear guard stops there, and
verifies the one-shot replan allowance without moving the real robot.
`TAI_CLEARANCE_MOTION=forward_route TAI_CLEARANCE_ADVANCE=0.15` keeps the robot
at the captured actual pose and validates the full joined departure plus
the matching hypothetical-start SE2 route. It does not move the robot.

`scripts/test_real_short_goal` is an explicitly authorized REAL motion test,
not a read-only probe. By default it sends one short forward goal with unchanged yaw
and cancels on stale sensors, excess travel, reverse, large rotation or timeout.
Use `python3 scripts/test_real_short_goal --distance 0.45` only with a clear
physical corridor and an operator ready to stop the robot. Cancel/stop uses
normal Nav2 and `/cmd_vel_nav`, never bypassing the safety chain.
`--heading` can supply an explicit map heading, and zero distance is supported
for a pure terminal-heading test; the same 35-degree rotation watchdog remains.

## Live check, 2026-09-26

With the edited real map, EKF and filtered LiDAR, the updated stack accepted
two consecutive short real goals after the earlier aborts. The 25 cm forward
goal succeeded with zero recoveries and 4.3 cm localization-based XY error.
The next 20 cm goal succeeded with one checked 15 cm forward escape and 2.4 cm
XY error. Neither test reversed or exceeded its rotation watchdog. The robot
was stopped afterwards. These are map/TF errors, not independent physical
accuracy measurements. Continuous BackUp cancellation is covered by the
isolated running-action regression, but a long real reverse was not tested.

## Stationary spin-collision regression, 2026-09-26

The operator's later goal ended after a checked retreat followed by a -0.52 rad
Spin: behavior_server reported COLLISION_AHEAD and the previous tree propagated
that directly into navigation abort. The updated tree instead checks the rear
and continues the same goal with measured additional retreat. Tests use mocked
motion actions in domain 231; no real goal was issued for this correction.

A stationary capture at approximately (-0.371, -0.230), heading 62.93 degrees,
was replayed with the last reported goal XY (3.40, 1.40) and an explicitly
hypothetical terminal yaw of zero (the RViz goal yaw was not captured).
Turning-pocket selection rejects the original pose and a 10 cm hypothetical
retreat, but accepts a complete useful turn/exit after a 20 cm hypothetical
retreat. This demonstrates local clearance, not execution success or proof
for the operator's exact terminal orientation. The physical vehicle stayed
stationary throughout these probes.

## Stopped-retreat replanning regression, 2026-09-26

A later live goal toward (3.32, -0.07) backed up for approximately 8 seconds
before the BT canceled BackUp and aborted, without a BackUp timeout or
collision result. A subsequent stationary capture near (0.067, -0.276),
heading -150.19 degrees, rejected local turn/forward escape and another
16 cm retreat, while a SE2 route passed full-footprint validation with an
explicitly hypothetical terminal yaw of zero.

The updated isolated replay verifies that a synthetic 15 cm measured retreat
to that captured pose now earns one stopped replan instead of immediate
abort. The SE2 route still passes the original validator. All three selected
test groups (stop envelope, rotation sweep, footprint BT) pass; the optional
capture replay was also run separately for both route validation and the
new one-shot allowance. These are no-motion tests, not proof of success for
the operator's original terminal heading. The live stack was not restarted
and no real navigation goal was sent for this change; the operator requested
restart commands to apply it themselves.

## Complete forward-route regression, 2026-09-26

After restart, two further goals still backed up and aborted. Logs showed
controller rotation collision stops, followed by the new stopped-retreat
replanning allowance; that allowance was active, but local turning-pocket
selection still rejected compound forward routes. A stationary capture at
approximately (-1.533, -0.621), heading -150.98 degrees, was probed toward
(1.23, -0.43) with explicitly hypothetical terminal yaw zero.

The updated `forward_route` replay keeps the robot at that original pose,
prepends the entire checked forward departure, and accepts the complete
SE2 routes through hypothetical forward starts 0.15, 0.45 and 0.60 m away.
The original start's SE2 route is still rejected at its unsafe intermediate
turn near (-2.633, -0.721); no collision cells or physical margins were removed.
Other tests cover length/turn ranking, trying the next route after geometric
execution failure, skipping a newly blocked choice, finite choice exhaustion,
planner cancellation/deadlines, fresh-data prefix checking past one metre,
wrong-goal rejection and clearing choices after success, failure or halt.
These tests never issue real velocity/navigation goals. Exact operator goal
orientation, dynamic obstacles and real tracking still need operator testing
after restart; success of every physically unreachable goal is not promised.

## Local oscillation regression, 2026-09-26

The live 16:29 launch logged two failed goals with repeated +/-15-degree
turns and 15 cm local advances, but no successful FollowPath execution during
those attempts. One goal aborted less than one second after reporting a
checked rear corridor. The subsequent goal did succeed; ordinary complete
route tracking is preserved by the region-memory change.

New isolated regressions cover two local adjustments surviving ordinary
replan/partial-turn resets, a single continuously running BackUp, release at
a different measured rear pose, immediate acceptance of a fully checked
goal-facing turn, a lost braking pocket, fault/rear-block rejection, one-shot
handoff consumption and new-goal memory reset. Pure helper tests cover signed
reverse displacement, sideways/forward rejection and an idempotent retreat
anchor using the actual reverse heading. Build succeeded and all three
selected CTest groups passed. The optional old capture replay was skipped
because its capture is not present. No real motion goal or launch restart
was performed for this revision; the operator must restart to load it.

## Selected route visualization

`/plan` contains the native planner trials. `/plan_selected` contains the
body-checked winner sent to FollowPath. The normal real/simulation display
relay subscribes to `/plan_selected` and clears `/rviz/plan` when navigation
finishes; `motion_test` retains the native `/plan` display.

## Three recent goals: isolated Gazebo replay, 2026-09-28

With the shared navigation parameters and the final direct-line gate, all
three recorded goals completed with zero Nav2 recoveries. The pillar crossing
from (1.99, 2.96) took 175 s, ending 3.94 cm and 3.63 degrees from the goal;
it needed one costmap-triggered replan. The two clear-area goals, replayed in
sequence from the measured first arrival, took 80 s and 112 s, ending 2.35 cm /
4.94 degrees and 1.78 cm / 4.81 degrees from their goals. The prior
shortest-line replay took 61 s and 159 s on the two clear-area goals; the
second now avoids the repeated terminal turns but takes longer. Gazebo timing
and localization can vary between launches, so these are specific replays,
not guaranteed times or evidence of hardware precision.

Only in the isolated headless replays, the collision monitor's camera scan
source was disabled at runtime because intermittent stale scan messages
stopped the simulation. The camera still fed the costmaps. The checked-in
camera collision-monitor configuration stays enabled for normal launch.

## Straight route with a late terminal pivot, 2026-10-02

The latest live goal started near (3.501, 0.513) at 90 degrees and ended near
(1.446, -0.379) at -170.7 degrees. Its straight XY bearing was -156.5 degrees.
The Direct candidate was rejected at its first accepted 5 cm XY point because
the final yaw sweep was blocked there, so the route search chose a longer
SE2Fallback curve. That check did not try the remaining few centimetres of
straight travel before pivoting at the exact endpoint.

For a Direct candidate, validation now checks the full-footprint straight
segment from the first accepted XY point to the endpoint and the complete yaw
sweep at that endpoint. The prepared-route check and FollowPath controller
allow that final translation when the current pivot is blocked and the
endpoint is ahead. The controller still checks its local costmap and stops if
the short segment becomes occupied. Synthetic tests cover acceptance of this
case, rejection of the same shape for a generic path, and straight motion
before the final turn. The two affected CTest groups pass. The available
costmap capture was taken after the goal, so it does not prove the original
start turn or straight corridor was clear at execution time. A live retry
after restarting Nav2 is needed to confirm that route on the robot.

## Rounded lattice corners, 2026-10-02

The next live goal from (3.708, 1.149) to (0.130, -0.569) selected a 4.205 m
SE2Fallback route. Its chosen path included a large initial turn and several
intermediate stop-turn-go corners of roughly 18-25 degrees. The Direct route
was blocked near (2.448, 0.544), so the screen's empty-looking center did not
establish that a straight body path was clear at planning time.

For current-pose SE2Fallback routes, the validator now tries a tangent circular
fillet at each small intermediate stationary corner. Radii of 0.40 or 0.30 m
are considered only when both neighboring straight segments fit the trim.
Every new arc and its joins are checked with the same global and nearby local
costmaps and padded body footprint used for the original route. A blocked or
too-tight corner keeps its original stop-turn-go primitive; the large initial
turn and last 30 cm near a goal are left alone. The selected route publishes
the actual modified path to `/plan_selected`, and the existing fresh-route
check validates it again during motion. Synthetic tests verified a continuous
clear corner and fallback when a cell blocks only the moving fillet. The three
relevant CTest groups passed. This is a local code change; the old live Nav2
process must be restarted before a robot trial can establish its effect.

## Pi 5 CPU and shortest route review, 2026-10-02

The state immediately before this change was pushed to the public repository
at `a992d59`. In a three to four second idle sample, the real Pi spent about
18% of one core in `bt_navigator`, 22% in the ESP32 IMU bridge, 19% in the
RViz-only IMU visualizer, and another 6-18% each across many independent
Nav2 processes. This is roughly two cores of combined CPU even without a
goal. The standard real launch started smoother, route, waypoint and docking
servers although its NavigateToPose tree does not call them.

The real launch now starts only controller, planner, behavior, velocity
smoother, collision monitor and BT navigator, with those six managed by the
navigation lifecycle manager. Localization and the physical safety command
chain remain active. The real BT loop checks at 20 Hz instead of 100 Hz;
the local costmap still updates at 8 Hz but publishes at 4 Hz rather than 8;
the RViz-only IMU markers publish at 5 Hz rather than 20; and the IMU bridge
polls its 50 Hz ESP32 feed at 100 Hz rather than 200 Hz. These numbers are
configuration changes, not measured CPU reductions: the current processes
continue using the old settings until an operator restart.

The route search now stops comparing planners after a validated current-pose
Direct candidate exactly matches the Euclidean start-to-goal XY lower bound.
Its normal fresh-footprint validation and collision monitor remain in place.
When Direct is blocked, the other planners still compete by checked route
length. A synthetic test verifies that a valid Direct candidate skips later
planner calls; existing ranked-route tests still cover the blocked/longer
alternatives.

For the pictured S path, the selected goal ran from about (3.389, 1.460) to
(3.094, -0.364) with final heading 180 degrees. A straight XY path would
travel at about -99 degrees, requiring an 81-degree final pivot. The live
planning log rejected Direct at the first accepted 5 cm XY point and chose
a 2.376 m lattice path versus 1.848 m straight XY distance. The costmap
captured after the goal also showed a lethal cell touched by the full padded
footprint during the exact-goal pivot. Because that capture is later than
planning, it is supporting geometry, not a replay of the original map. The
new route preference therefore does not force this straight line when the
specified 5-degree final heading cannot be reached safely.

## Near-goal rotation and retreat, 2026-10-04

For a goal only 3.4 cm from the robot, the October 4 log shows a direct
terminal turn blocked after it began, followed by a 0.606 m forward detour.
The detour stopped where the rear corridor was blocked and planning then
reported `Start occupied`. Terminal validation and the controller now check
both complete rotation directions. The controller keeps a chosen long turn
through subsequent control ticks. If both directions are blocked at an
already reached goal XY, a checked 16 cm rear corridor takes precedence over
forward detours, and continuous retreat seeks a safe turning pocket. A blocked
rear corridor still forbids reverse motion. The 5 cm XY and 5 degree yaw goal
tolerances are unchanged.

The first physical replay of the latest RViz goal, (3.607, 1.284) at
-1.598 rad, still aborted from (3.085, 1.610): a static-map lethal cell lay
inside the front fork side of the current footprint although the live local
lidar costmap was clear. An isolated replay of the captured costmaps reproduced
the refusal to reverse. The retreat check now permits release of that existing
map-only contact in a private validation map only when it is in the front fork
side or extra footprint padding, the local map is clear, and the full reverse
corridor and original-map endpoint are clear. A nearby goal behind the chassis
also reaches this checked recovery branch when the planner reports bounded
search timeout 207. These conditions do not clear the published costmap or
permit reverse through a new obstacle.

After rebuilding and restarting Nav2, a second live replay of the same goal
succeeded in 45.6 s. The robot reversed about 0.16 m, replanned from the new
pose, and followed a checked 0.451 m route. The final stationary TF was about
(3.597, 1.256), yaw -1.658 rad, approximately 3.0 cm and 3.5 degrees from
the requested pose. The relevant CTest groups passed before the physical run.

## Short heading goal interrupted during turn, 2026-10-04

The latest RViz goal was (3.237, 1.429), yaw -1.564 rad, just 2.8 cm from
the starting XY (3.26, 1.41). Nav2 correctly selected a Direct 2.8 cm path,
but stopped its turn after about eight seconds as the estimated XY shifted to
(3.318, 1.361). The prepared-path validator then rechecked the old initial
heading instead of the remaining part of the turn. A 55 cm retreat and a
1.079 m SE2Fallback detour followed; the goal ultimately aborted. The map
capture after the event suggests the short turn passes a static-map wall,
and the opposite long turn meets other obstacles. The later capture is not
proof of the exact obstacles present during the goal.

For a Direct goal whose initial XY is already within the 5 cm tolerance, the
prepared-path check now validates the remaining yaw sweep from the robot's
actual pose during a bounded 12 cm pivot drift. It still checks both current
costmaps and the full body. Once yaw is reached or drift exceeds that bound,
the normal path validation applies; the 5 cm / 5 degree goal checker remains
unchanged. A regression test reproduced rejection of a clear remaining sweep
and verifies that a newly occupied cell on that sweep still stops the turn.
The full footprint BT test group passed locally, with one unrelated optional
capture replay skipped.

A first physical yaw-only retry from (3.250, 1.631), yaw 197.3 degrees, to
-89.6 degrees stayed still and aborted after 27.2 s. A fresh global/local
costmap capture and isolated replay found an existing static-map lethal cell
under the front fork tip, although the local lidar costmap was clear there.
During the first few degrees of the positive turn, rasterization brought two
more cells of the same map wall under the fork. The negative long turn met
other occupied cells. The terminal pivot check now releases existing map-only
cells in the extra footprint padding, plus fork-tip cells in the first
0.25 rad of the sweep within 0.70-0.86 m forward and 0.16 m laterally, only
when those cells are clear in the live local map. The original global map
must be clear after that initial
sweep and at the goal; every live local obstacle still blocks the full turn.
The published costmaps remain unchanged. Isolated replay of the exact captured
maps then accepted the positive 73.1 degree turn. Synthetic tests kept a
new obstacle in the later sweep, a locally observed fork contact, unknown
cells, and core-body contacts outside the fork tip blocked.

After installation and Nav2 restart, a new yaw-only physical test at the
current XY (3.251, 1.632) reached -89.6 degrees in 5.2 s with zero recoveries.
The maximum XY displacement in feedback was 3.6 cm. Stationary TF afterward
was (3.289, 1.645), yaw -91.4 degrees: approximately 4.0 cm and 1.8 degrees
from that requested pose. The full footprint BT test group passed: 106
tests, one optional capture test skipped; the captured-costmap replay also
passed separately.

## Straight route after a checked departure, 2026-10-04

The tested rotation/retreat state was pushed as `57b17be` before this change.
The latest pictured successful goal ran from approximately (0.855, -0.442)
to (3.500499, 0.369664), yaw 1.586412 rad. Direct was rejected at its launch
turn; subsequent departures competed only through GridBased, SE2Arc and
SE2Fallback. Multiple costmap updates stopped selected routes. After a retreat,
the final successful SE2Arc route measured 4.611 m. A later global/static/lidar
capture showed the straight translation corridor clear; it does not establish
that the launch or terminal turn was clear during planning.

Each checked forward departure now tries an explicit stationary turn followed
by a dense straight segment to the exact goal before requesting a planner.
The actual forward prefix, pocket turn, full-body translation and terminal
yaw are validated together with the normal global and nearby local costmaps.
The route participates in the same length ranking. When it fits, later planner
calls at that same departure are skipped because they cannot shorten its
straight remainder; other departure distances are still compared. If it does
not fit, existing planner alternatives remain available. No footprint, obstacle
threshold or goal tolerance was reduced.

A synthetic obstacle that blocks the initial pivot but leaves a forward
pocket clear verified a 0.60 m departure plus exactly 1.00 m straight remainder.
Fresh obstacles on that remainder or under the actual launch body rejected
the candidate. The full footprint BT suite and real Nav2 XML construction
passed. The pictured goal has already finished; this new departure candidate
has not been physically replayed from its original starting pose.

## Near-goal pivot direction, 2026-10-06

The latest yaw-only goal began near (3.66, 0.59) and requested yaw -1.5503 rad.
The previous Direct route accepted a short pivot after releasing an existing
map-only fork-tip contact. About five seconds later, a few centimetres of pose
change made the remaining footprint sweep fail; repeated route searches found
no checked path and Nav2 aborted. The log does not prove which physical object
the fork touched, but it does show that the first pivot validation was too
fragile for this position.

At an already accepted goal XY, the BT now checks both complete signed turns
with the full body, 4 cm pose shifts and a 0.06 rad stopping margin. It sends
the selected signed angle to Nav2 Spin, so the executed direction is the one
validated against the global and live local costmaps. If neither direction
fits, the existing checked recovery inspects the rear corridor. A collision
reported by Spin excludes another attempt at the same near-goal pivot before
the robot has moved. The 5 cm / 5 degree goal tolerances are unchanged.
The synthetic blocked-side test and the complete BT test group passed:
108 passed, one optional replay skipped. Nav2 was rebuilt and restarted;
this particular physical goal has not been replayed yet.

Follow-up after the next live goals: the 4 cm pivot reserve rejected the
nearby turns, and recovery also rejected reverse at the same fixed pose.
A read-only capture at (3.669, 0.565), yaw -20.5 degrees showed six lethal
static-map cells under the centre of the front fork tip (0.71-0.78 m ahead),
while the live local costmap had no lethal cell inside the footprint. The
retreat map-contact release previously covered the fork sides but not the
centre tip, although the pivot check already covered this tip. Checked reverse
now admits only these existing map-only tip cells when the live local cells
are clear; the complete retreat corridor and endpoint on the original map
must still be free. Captured-costmap replay accepts reverse from the actual
pose and from hypothetical positions 5 cm and 10 cm farther back. A synthetic
test confirms a newly observed local obstacle at the tip still stops retreat.
Nav2 was restarted after the build to load this correction.

Live follow-up on 2026-10-06: a yaw-only goal still aborted without motion.
The near-goal sweep requested checked retreat, but `route_control_available`
and the later unprepared path probe cleared `terminal_escape_needed` before
`recovery_allowed` could reach BackUp. Those route probes now preserve the
pending escape request. A regression test exercises the complete sequence
from a blocked near-goal turn through route selection to `reverse_needed`.
The full BT suite passed (109 passed, one optional capture replay skipped).

After rebuilding and restarting Nav2, a bounded real goal 0.30 m behind the
stationary robot at (3.669, 0.563), yaw -20.5 degrees, used continuous checked
retreat. The robot moved about 0.25 m and Nav2 reported `Goal succeeded` at
(3.428, 0.648), 4.4 cm from the requested XY. A second bounded yaw-only goal
at that position requested -0.75 rad; the checked sweep selected the negative
23.2-degree turn, and Nav2 reported `Goal succeeded` in about two seconds at
yaw -0.771 rad, with about 2 cm XY drift. No GitHub push was made.
