var storedPlayerScripts = {
    random1: {
        displayName: "Random1",
        playerClass: function() {
            var ownData = undefined;
            
            this.init = function(c, playerdata, selfindex){
                console.log('hello, world');
                // here an initialization might take place;
            }
            
            this.moveFunction = function(c, playerdata, selfindex){
                var self = playerdata[selfindex]; // read the info for the actual player
                var newcenter = { // thats how the center of the next movement can be computed
                    x: self.pos.x+(self.pos.x-self.oldpos.x),
                    y: self.pos.y+(self.pos.y-self.oldpos.y)
                };
                var nextmove=newcenter; 
                // the variable nextmove is initialized as the center point
                // if it is valid, we stay there with a high probability
                if (!lc.equalPoints(newcenter,self.pos) && lc.validLine(self.pos, newcenter) && lc.playerAt(newcenter)<0 && Math.random()>0.1)
                    return {x: 0, y: 0}; // with returning 0,0, the next movement will be the center
                else { // the center point is not valid or we want to change with a small probability
                    var validmoves = [];
                    var validstay = null;
                    // we try the possible movements
                    for(var i=-1; i<=1; i++)
                        for(var j=-1; j<=1; j++){
                            nextmove={x: newcenter.x+i, y: newcenter.y+j};
                            // if the movement is valid (the whole line has to be valid)
                            if(lc.validLine(self.pos, nextmove) && (lc.playerAt(nextmove)<0 || lc.playerAt(nextmove)==selfindex))
                                if(!lc.equalPoints(nextmove,self.pos)) // if there is no one else
                                    validmoves.push({x: i, y: j}); // we store the movement as a valid movement
                                else
                                    validstay={x: i, y: j}; // the next movement is me
                        }
                    if(validmoves.length){
                        // if there is a valid movement, try to step there, if it not equal with my actual position
                        return validmoves[Math.floor(Math.random()*validmoves.length)];
                    }
                    else{
                        // if the only one movement is equal to my actual position, we rather stay there
                        if(validstay){
                            return validstay;
                        }
                    }
                    return {x: 0, y: 0}; // if there is no valid movement, then close our eyes....
                }
            }
        }
    },
};