# ------------------------------------------------------------------------ #
#      o-o      o                o                                         #
#     /         |                |                                         #
#    O     o  o O-o  o-o o-o     |  oo o--o o-o o-o                        #
#     \    |  | |  | |-' |   \   o | | |  |  /   /                         #
#      o-o o--O o-o  o-o o    o-o  o-o-o--O o-o o-o                        #
#             |                           |                                #
#          o--o                        o--o                                #
#                        o--o      o         o                             #
#                        |   |     |         |  o                          #
#                        O-Oo  o-o O-o  o-o -o-    o-o o-o                 #
#                        |  \  | | |  | | |  |  | |     \                  #
#                        o   o o-o o-o  o-o  o  |  o-o o-o                 #
#                                                                          #
#    Jemison High School - Huntsville Alabama                              #
# ------------------------------------------------------------------------ #

from datetime import datetime

from lib_6107.util.competition import add_event, Event, EventStartEndTime

# The format string must match the input string exactly
_format_string = '%Y-%m-%d %H:%M %z'

add_event(Event("Rocket City Regionals",
                (EventStartEndTime(datetime.strptime("2026-04-08 08:00:00 CDT", _format_string),
                                   datetime.strptime("2026-04-12 17:00:00 CDT", _format_string)))))

add_event(Event("Rotary Club of Greater Huntsville",
                (EventStartEndTime(datetime.strptime("2026-04-15 09:00:00 CDT", _format_string),
                                   datetime.strptime("2026-04-12 15:00:00 CDT", _format_string)))))
