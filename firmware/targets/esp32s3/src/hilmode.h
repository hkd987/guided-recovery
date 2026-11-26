// Gate 4 bench mode: the PC simulator (sim/apps/grhil.cpp) flies the flight
// over USB serial; this board makes every decision and drives its real
// winches. Entered with the console command 'hil' (magnet on); leave by
// rebooting. The board's own magnet is a physical interlock: the winches
// only move once it is removed.
#pragma once

#include "gr/flight_core.hpp"

bool hilActive();
// Called from the console. Returns false (and says why) unless disarmed.
bool hilEnter(gr::FlightCore& core);
// Called from the flight task instead of the normal sensor cycle.
void hilService();
