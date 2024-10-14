/*
    wait for planning scene 
    creates an array of bounding xyz values for the collision objects (+ a little tolerance)
    create service server
    service cb: given the possible bp pose checks if it is within bounds of any collision object
                if it is it returns false (pose is not valid for bp)
                if it is not returns true (pose is valid for bp)

   the client is in place base, in the "findbase..." functions:
   when a probable pose is found, in addition to the other checking it already does, before storing it into the base_poses array 
   it calls for this server, if it returns false it does not save the pose

    
*/