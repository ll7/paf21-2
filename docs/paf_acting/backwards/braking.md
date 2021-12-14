# What is the fastest way to break? (e.g. Emergency Break)

Tests at 100 km/h
|handbreak|break|throttle|steer|reverse||result (s)|
|---|---|---|---|---|---|---|
|1|-1||30°|||1.05|
|1||1|30°|1||1.2|
|1|-1|1|30°|1||1.25|
||-1||30°|||1.3|
|1||1||1||1.3|
|1|-1|||||2.1|
|1||||||2.7|
||-1|||||2.7|
|1||1||||NaN|
|||1||1||NaN|
|1||-1||||NaN|
|1|-1|-1||||NaN|

With handbreak, brake and steer, we get the fastest emergency break time. Problem: We sheer off and need more time to get back on track, in this manever we could hit someone beside us.

Solution:

We use handbrake with throttle, 30° steering and revese. This way, we don't sheer off and we are orented the way we approached -> Note that we need to release the emergency mode, or else we will drift to the right (carla bug), could be used to park sideways
