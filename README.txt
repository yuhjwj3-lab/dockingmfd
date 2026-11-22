DOCK MFD for Orbiter 2016  (autonomous rendezvous + docking, separate from ARLC)

BUILD (same as the Weather MFD)
 1. New GitHub repo, upload this whole folder (keep the hidden .github folder).
 2. Copy Orbiter's Orbitersdk\include and Orbitersdk\lib into the repo's sdk\ folder (sdk\include, sdk\lib).
 3. Actions -> Build DockMFD -> Run workflow -> download DockMFD-install.
 4. Copy Modules\Plugin\DockMFD.dll to C:\Orbiter2016\Modules\Plugin\ , tick DockMFD in Launchpad > Modules.
 5. In flight: MFD mode "Dock" (key D when choosing the MFD mode).

USING IT
 You must be in a stable orbit (periapsis above the atmosphere). Targets = every other vessel orbiting the SAME body
 that has a FREE docking port (ISS, other ships...). Turn OFF any other autopilot (XR Attitude Hold, Descent Hold,
 navmodes) before engaging, keep RCS switched on, and keep time acceleration at 1x when you press ENG.
   UP / DN   move the cursor in the target list
   SEL       select the highlighted target
   PRT / TPT choose which docking port of YOUR ship / of the TARGET to use (cycles through the free ports)
   STO       standoff distance of the approach point on the target's port axis (1 / 2 / 5 km)
   ECO/FST   plan style: ECO = least fuel (can mean waiting several orbits), FST = more fuel, shorter wait
   PLN       compute the plan (a few seconds; shows burn 1 + burn 2 delta-v and times)
   ENG       press TWICE within 8 s to start
   ABT       cancel / abort - you get all controls back immediately
   WRP       auto time-warp for the long coasts (warp is always limited before burns, even when WRP is off)
   PG        switch between setup page and status page
 Keys: UP DOWN S P E A W G.

WHAT IT DOES
 Plan: searches departure time, transfer time and number of revolutions (multi-revolution Lambert solver) for the cheapest
 two-burn transfer to a point on the target's port axis, rejects transfers that dip into the atmosphere.
 Flight: RCS check (tiny pulses to learn which way each RCS group turns the ship) -> coast -> burn 1 (main engines) ->
 coast with mid-course corrections -> braking burn -> docking funnel (slow approach along the port axis, ship port pointed
 at the target port, roll aligned, speed shrinking with range) -> Orbiter docks automatically when the ports meet.
 If you are already within 12 km and slow, it skips the long-range part and goes straight to the funnel.
 It goes round the target instead of through it when repositioning.

NEEDS from your ship: main engines (long range only), linear RCS in all six directions, one free docking port.
Works with any vessel that defines these (Delta-glider, XR2, XR5, ...).

IMPORTANT / HONEST
 Written and tested OUTSIDE Orbiter: the orbital maths, planner and control laws were run in an offline simulation
 (test\ folder) and the Orbiter glue was only syntax-checked. It has never flown in the real simulator, so expect to
 need a round or two of tuning. Please send DockMFD_flight.log (created in the Orbiter folder) after a try.
 Known limits: Orbiter's J2 gravity and drag are not modelled in the plan (mid-course corrections absorb them);
 plane changes of many degrees cost a lot of delta-v and may show "NO SAFE PLAN"; if the first burn orientation looks
 wrong press ABT and tell me what you saw.
