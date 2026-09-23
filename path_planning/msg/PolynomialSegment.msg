# Position c0+c1*t+c2*t^2+c3*t^3, local t in seconds; axes x/y/z in header frame.
# The frame is supplied by the containing PlanResult. Position is in meters.
# Velocity=c1+2*c2*t+3*c3*t^2; acceleration=2*c2+6*c3*t.
# Concatenate segments by summing durations; p/v are continuous, acceleration may jump.
float64 duration
float64[4] x
float64[4] y
float64[4] z
