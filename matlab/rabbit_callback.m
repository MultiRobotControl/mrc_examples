function rabbit_callback(~, msg)
global RABBIT_POSITION
RABBIT_POSITION = msg;
return