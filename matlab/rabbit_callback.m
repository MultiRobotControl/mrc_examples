function rabbit_callback(~, msg)
global RABBIT_POSITION
RABBIT_POSITON = msg;
return