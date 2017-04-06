v 20130925 2
C 40000 40000 0 0 0 title-B.sym
C 40800 43800 1 0 0 connector4-1.sym
{
T 42600 44700 5 10 0 0 0 0 1
device=CONNECTOR_4
}
C 48700 48300 1 180 0 connector5-1.sym
{
T 46900 46800 5 10 0 0 180 0 1
device=CONNECTOR_5
}
C 48700 43700 1 180 0 connector5-1.sym
{
T 46900 42200 5 10 0 0 180 0 1
device=CONNECTOR_5
}
N 43600 42600 43600 47200 4
N 43600 47200 47000 47200 4
N 43600 42600 47000 42600 4
N 47000 47500 44200 47500 4
N 44200 42900 44200 47500 4
N 44200 42900 47000 42900 4
N 42500 44900 43600 44900 4
N 42500 44000 44200 44000 4
N 47000 47800 43100 47800 4
N 43100 44600 43100 47800 4
N 43100 44600 42500 44600 4
N 47000 43200 43100 43200 4
N 43100 43200 43100 44300 4
N 43100 44300 42500 44300 4
C 55300 49800 1 90 1 led-2.sym
{
T 54700 49700 5 10 0 0 270 2 1
device=LED
}
C 54300 49100 1 0 1 photo-transistor-1.sym
{
T 54500 49600 5 6 0 1 0 6 1
device=PS2501-1
T 54340 49100 5 10 0 1 0 6 1
device=photo-transistor
}
C 48900 48300 1 180 1 connector5-1.sym
{
T 50700 46800 5 10 0 0 180 6 1
device=CONNECTOR_5
}
N 55200 49800 55200 50100 4
N 55200 50100 56200 50100 4
N 56200 50100 56200 47200 4
N 56200 47200 50600 47200 4
N 54100 49100 54100 47500 4
N 50600 47500 55200 47500 4
N 55200 48900 55200 47500 4
N 54100 49500 54100 50100 4
N 53100 50100 54100 50100 4
N 53100 47800 53100 50100 4
N 53100 47800 50600 47800 4
C 50600 48000 1 0 0 inductor-1.sym
{
T 50800 48500 5 10 0 0 0 0 1
device=INDUCTOR
T 50800 48700 5 10 0 0 0 0 1
symversion=0.1
}
N 51500 48100 51500 46900 4
N 51500 46900 50600 46900 4
N 47000 46900 45800 46900 4
N 45800 46900 45800 48900 4
B 48900 46100 7600 4500 3 0 0 0 -1 -1 0 -1 -1 -1 -1 -1
C 45600 50600 1 270 0 connector2-1.sym
{
T 46600 50400 5 10 0 0 270 0 1
device=CONNECTOR_2
}
C 46300 40400 1 90 0 connector2-1.sym
{
T 45300 40600 5 10 0 0 90 0 1
device=CONNECTOR_2
}
N 47000 48100 46100 48100 4
N 46100 48100 46100 48900 4
N 47000 43500 45800 43500 4
N 45800 43500 45800 42100 4
N 47000 42300 46100 42300 4
N 46100 42300 46100 42100 4
T 52000 46400 9 10 1 0 0 0 1
XRobot wheel motor A
T 50500 48300 9 10 1 0 0 0 1
motor windings
T 54100 50300 9 10 1 0 0 0 1
speed sensor
T 45000 50700 9 10 1 0 0 0 1
to Raspberry Pi Motor Hat
T 44800 40100 9 10 1 0 0 0 1
to Raspberry Pi Motor Hat
T 40300 45300 9 10 1 0 0 0 1
to 8051 speed sensor interface
T 49600 47500 9 10 1 0 0 0 1
black
T 49600 47200 9 10 1 0 0 0 1
brown
T 49600 47800 9 10 1 0 0 0 1
white
T 49600 48100 9 10 1 0 0 0 1
blue
T 49600 46900 9 10 1 0 0 0 1
red
T 48900 42800 9 10 1 0 0 0 1
to XRobot wheel motor B
T 51500 40900 9 10 1 0 0 0 1
Wheel motor interface cable wiring
T 50300 40400 9 10 1 0 0 0 1
wheel_motor.sh
T 50500 40100 9 10 1 0 0 0 1
1
T 52000 40100 9 10 1 0 0 0 1
1
T 54800 40400 9 10 1 0 0 0 1
1.0
T 54800 40100 9 10 1 0 0 0 1
Ian
