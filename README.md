planlogging
===========

Plan Logger for recording events from different sources and outputting them into .dot or .owl format. Also, it has an elaborate ROS service interface.

The Plan Logger node can record data from arbitrary sources, building up one of the following two:
 * A tree hierarchy of events that are associated in parent/child connections depending on when they arrive in .dot format
 * a Gantt-like output data structure in .owl format
