Controls the led drivers based on the input ros message. 
The message is a percentage represented from 0-100. 
Any value outside of this range will be truncated to the nearest value in 
range (i.e -10 will be forced to 0).

The publisher will simply provide the current state of the led's
