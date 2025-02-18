myStruct = struct();
myStruct.timestamp        = 0;
myStruct.position         = [0;0;0];
myStruct.velocity.x       = single(0);
myStruct.velocity.y       = single(0);
myStruct.velocity.z       = single(0);
myStruct.rotationMatrix   = eye(3);
myStruct.status.isValid   = false;
myStruct.status.bitfield  = uint16(0);
myStruct.status.errorCode = uint8(0);

BusSerializer.StructToSubsystems(myStruct, 'myBus', 'Example');
