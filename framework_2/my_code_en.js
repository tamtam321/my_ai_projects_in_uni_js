typeof Infinity;
typeof -Infinity;

let map = [];
let heuristic_map = [];
let players;
let my_player;
let self_idx;
let init_pos;
let init_state;
let is_used_rec = [];	// Matrix where each cell tells you which one has been already used for recursive call.
let curr_min_heu_val = Infinity;	// Tracking the current smallest heuristic value to know when it reaches the goal.
let goal_val = Infinity;

let path = [];	// Path from current state to goal state.
let step_history = [];	// Step history

/**
 * Indicate when the agent is not able to take valid step and has to collide with wall.
*/
let did_it_collide = false;

// Print heuristic map
function printHeuMap()
{
	let str = "";
	let map_length = heuristic_map.length * heuristic_map[0].length;
	let tmp = "";

	for(let i = 0; i < heuristic_map.length; i++)
	{
		for(let j = 0; j < heuristic_map[0].length; j++)
		{
			if(heuristic_map[i][j] === -Infinity)
			{
				tmp = "x";
			}
			else
			{
				tmp = heuristic_map[i][j].toString();
			}

			while(tmp.length !== (map_length.toString().length + 1))
			{
				tmp += " ";
			}

			str += tmp;
		}

		str += "\n";
	}

	str = "\n" + str;

	console.log(str);
	console.log();
}

// Print map
function printMap()
{
	let str = "";
	let map_length = map.length * map[0].length;
	let tmp = "";

	for(let i = 0; i < map.length; i++)
	{
		for(let j = 0; j < map[0].length; j++)
		{
			if(typeof map[i][j] === "undefined")
			{
				tmp = "?";
			}
			else if(map[i][j] < 0)
			{
				tmp = "x";
			}
			else
			{
				tmp = map[i][j].toString();
			}

			while(tmp.length !== (map_length.toString().length + 1))
			{
				tmp += " ";
			}

			str += tmp;
		}

		str += "\n";
	}

	str = "\n" + str;

	console.log(str);
	console.log();
}

/**
 * Check if the agent is on the track.
*/
function onTrack(x, y)
{
	return x >= 0 && y >= 0 && x < map.length && y < map[0].length && map[x][y] >= 0;
}

/**
 * Check if the agent is in the map.
*/
function inMap(x, y)
{
	return x >= 0 && y >= 0 && x < map.length && y < map[0].length;
}

/**
 * Is the point is in the range of sight or is it known?
*/
function isPointDefined(x, y)
{
	return typeof map[x][y] !== "undefined";
}

// Heuristic value calculator (recursive method)
function heuCal(x, y, hvalue)
{
	if(!is_used_rec[x][y])
	{
		heuristic_map[x][y] = hvalue;
		is_used_rec[x][y] = true;
		let valid_steps = [];	// Stores cells which haven't been used yet for calculating its neighbours' heuristic value.

		if(hvalue < curr_min_heu_val)
		{
			curr_min_heu_val = hvalue;
		}

		if(isPointDefined(x, y))	// Is the current point is known?
		{
			// Checking current cell's neighbours
			// and calculate theirs heuristic value.
			for(let i = -1; i <= 1; i++)
			{
				for(let j = -1; j <= 1; j++)
				{
					/**
					 * Don't need to consider the current cell, because it has been already checked.
					*/
					if(!(i === 0 && j === 0))
					{
						let step_x = x + i;
						let step_y = y + j;

						if(isPointDefined(step_x, step_y) && onTrack(step_x, step_y))
						{
							if(map[step_x][step_y] === 100)
							{
								heuristic_map[step_x][step_y] = 0;
								valid_steps.push([step_x, step_y, 0]);
							}
							else if(hvalue - 1 > heuristic_map[step_x][step_y])
							{
								heuristic_map[step_x][step_y] = hvalue - 1;
								valid_steps.push([step_x, step_y, hvalue - 1]);
							}
						}
					}
				}
			}
		}

		/**
		 * Continue calculating the next cell's neighbours heuristic values.
		*/
		for(let k = 0; k < valid_steps.length; k++)
		{
			let step = valid_steps[k];
			heuCal(step[0], step[1], step[2]);
		}
	}
}

/**
 * Clear path, step history.
 * Each step I rebuild the path and create new step history.
 * */ 
function clearPathHis()
{
	path = [];
	step_history = [];
}

// Return with the cell heruristic value.
function heuVal(state)
{
	return heuristic_map[state.cs.x][state.cs.y];
}

/**
 * Because it couldn't see the whole map at the beginning, so it is gonna use the 
 * heuristic matrix to estimate the goal direction until it reaches the goal.
 * The heuristic value of the goal is 0.
*/
function isGoal(state)
{
	return heuristic_map[state.cs.x][state.cs.y] === goal_val;
}

function mapNeighbours(state)
{
	let neighbour_states = [];	// List of cells where the agent could step.

	// list -> second best options are those state which collide with other player, if there is no valid options.
	let bad_choices_collision_player = [];
	
	// list -> worst case, there is no valid and second options so what left is to collide with wall.
	let bad_choices_collision_wall = []

	let newCenter = 
	{
		x: state.cs.x + (state.cs.x - state.ps.x),
		y: state.cs.y + (state.cs.y - state.ps.y)
	}

	for(let i = -1; i <= 1; i++)
	{
		for(let j = -1; j <= 1; j++)
		{
			let next_step = {x: newCenter.x + i, y: newCenter.y + j};

			/**
			 * If next step is valid and stays on the track.
			*/
			if(lc.validVisibleLine(map, state.cs, next_step) && heuristic_map[next_step.x][next_step.y] !== -Infinity)
			{
				let new_state = 
				{
					cs: next_step,
					ps: 
					{
						x: state.cs.x,
						y: state.cs.y
					},
					parent: state,
					w: 1	// weight, each step add 1 weight.
				};
				
				// Is the step valid?
				if((lc.visiblePlayerAt(map, next_step) < 0 || lc.visiblePlayerAt(map, next_step) === self_idx))
				{
					neighbour_states.push(new_state);	// Stores valid steps.
				}
				else
				{
					bad_choices_collision_player.push(new_state);	// Stores steps which causes collision with player. (second option)
				}
			}
			else if(inMap(next_step.x, next_step.y)) // If it couldn't avoid the collision with the wall then it has to go into it.
			{
				let new_state_bad = 
				{
					cs: next_step,
					ps: 
					{
						x: state.cs.x,
						y: state.cs.y
					},
					parent: state,
					w: 1	// weight, each step add 1 weight.
				};

				bad_choices_collision_wall.push(new_state_bad)
			}
		}
	}

	// If it is unable to make valid move then chooses invalid move.
	if(neighbour_states.length === 0)
	{
		// Check if there are second options available.
		// If not then collide with wall.
		if(bad_choices_collision_player.length === 0)
		{
			did_it_collide = true;

			for(let i = 0; i < bad_choices_collision_wall.length; i++)
			{
				neighbour_states.push(bad_choices_collision_wall[i]);
			}
		}
		else
		{
			for(let i = 0; i < bad_choices_collision_player.length; i++)
			{
				neighbour_states.push(bad_choices_collision_player[i]);
			}
		}
	}

	return neighbour_states;
}

function areStatesEqual(s1, s2)
{
	return s1.cs.x === s2.cs.x && s1.cs.y === s2.cs.y && s1.ps.x === s2.ps.x && s1.ps.y === s2.ps.y;
}

/**
 * True when the state isn't in the list (list of possible steps) or
 * if it is, but the new state has better f values.
*/
function isAcceptableState(list, state)
{
	for(let i = 0; i < list.length; i++)
	{
		if(areStatesEqual(list[i], state))
		{
			return state.f < list[i].f;
		}
	}

	return true;
}

/**
 * Does the list contain the state?
*/
function isContain(list, state)
{
	for(let i = 0; i < list.length; i++)
	{
		if(areStatesEqual(list[i], state))
		{
			return true;
		}
	}

	return false;
}

function sortList(list)
{
	return list.sort(function(a, b)
	{
		if(a.f < b.f)
		{
			return -1;
		}
		if(a.f > b.f)
		{
			return 1;
		}
		return 0;
	});
}

/**
 * From goal it propagates back to the current state and build up the path.
*/
function backpropagation(goal_state)
{
	let state = goal_state;

	path = [];

	while(state.parent != null)
	{
		path.push(state);
		state = state.parent;
	}

	path.reverse();
}

// A* search algorithm
function aStar(state)
{
	let timer_start = performance.now();
	let timer_sum = 0;

	clearPathHis();

	state.f = heuVal(state);
	let possible_steps = [];	// list of possible steps (also contains second option steps).
	possible_steps.push(state);
	let no_choice_bad_steps = [] // Wall collison cases.
	let goal_state = null;	// Here I'm gonna store the goal state.


	/**
	 * If there are valid steps then use them to search for the goal state.
	 * Because It has limited vision, so at the beginning it searches for the smallest heuristic value
	 * in the vision, so it tries to heading the goal direction which has value 0.
	 * 
	 * Also If the agent couldn't step valid steps then it chooses the second option which
	 * leads to collide with other player or if there is no second option then the next possible
	 * step is leading into wall.
	*/
	while(possible_steps.length > 0 && timer_sum < 900)
	{
		let curr_state = possible_steps.shift();
		step_history.push(curr_state);
		goal_state = curr_state;

		if(isGoal(curr_state))
		{
			break;
		}

		let neighbour_states = mapNeighbours(curr_state);

		for(let i = 0; i < neighbour_states.length; i++)
		{
			neighbour_states[i].f = (curr_state.f - heuVal(curr_state)) + (neighbour_states[i].w + heuVal(neighbour_states[i]));	// f = g + h
		}

		if(did_it_collide)
		{
			for(let i = 0; i < neighbour_states.length; i++)
			{
				if(isAcceptableState(no_choice_bad_steps, neighbour_states[i]) && !isContain(step_history, neighbour_states[i]))
				{
					no_choice_bad_steps.push(neighbour_states[i]);
				}

				neighbour_states.shift();
				i = -1;
			}

			no_choice_bad_steps = sortList(no_choice_bad_steps);

			/**
			 * Set it back to false, the program noted that bad cases are stored and 
			 * use it when necessary. Later if again it has to store another bad cases then it could
			 * warn the program.
			*/
			did_it_collide = false;

			/**
			 * If there is valid step then deal with it first. Need this because if it lets this loop
			 * move on then wall collision cases are gonna store among valid and second option steps.
			*/
			if(possible_steps.length > 0)
			{
				continue;
			}
		}

		if(possible_steps.length > 0 || neighbour_states.length > 0)
		{
			/** 
			 * It checks many possibilities of steps and based on which one is valid and has better f values 
			 * it's gonna find the goal and from there builds up the path.
			 * */ 
			for(let i = 0; i < neighbour_states.length; i++)
			{
				/**
				 * Is the current state has already in the list of possible steps or history?
				 * 
				 * If it hasn't been in the list, then examine.
				 * If it has been int the list, then compare the f values which one has better.
				 * If it has better value, then examine it, otherwise drop it.
				*/
				if(isAcceptableState(possible_steps, neighbour_states[i]) && isAcceptableState(step_history, neighbour_states[i]))
				{
					possible_steps.push(neighbour_states[i]);
				}
			}
		}
		else	// If there is no valid and second option step then it checks the worst case.
		{
			for(let i = 0; i < no_choice_bad_steps.length; i++)
			{
				if(isAcceptableState(possible_steps, no_choice_bad_steps[i]) && !isContain(step_history, no_choice_bad_steps[i]))
				{
					possible_steps.push(no_choice_bad_steps[i]);
				}
			}
		}
		

		possible_steps = sortList(possible_steps);	// Sort by heuristic value, it is gonna take the better one first.

		let timer_end = performance.now();
		timer_sum = timer_end - timer_start;
	}

	backpropagation(goal_state);	// Got the goal and build up the path.

	return goal_state;
}

/**
 * Search for the current smallest heuristic value with the current vision of the map.
*/
function searchForCurrMinHeuVal()
{
	curr_min_heu_val = Infinity;

	for(let i = 0; i < heuristic_map.length; i++)
	{
		for(let j = 0; j < heuristic_map[0].length; j++)
		{
			if(heuristic_map[i][j] !== -Infinity && heuristic_map[i][j] < curr_min_heu_val)
			{
				curr_min_heu_val = heuristic_map[i][j];
			}
		}
	}
}

/**
 * Update Map
*/
function updateMap(fresh_map)
{
	for(let i = 0; i < map.length; i++)
	{
		for(let j = 0; j < map[0].length; j++)
		{
			if(typeof map[i][j] === "undefined" && typeof fresh_map[i][j] !== "undefined")
			{
				map[i][j] = fresh_map[i][j];
			}
		}
	}
}

/**
 * Heuristic matrix update.
*/
function updateHeuMap()
{
	is_used_rec = [];

	for(let i = 0; i < map.length; i++)
	{
		is_used_rec.push([]);

		for(let j = 0; j < map[0].length; j++)
		{
			is_used_rec[i].push[false];
		}
	}

	for(let i = 0; i < heuristic_map.length; i++)
	{
		for(let j = 0; j < heuristic_map[0].length; j++)
		{
			if(heuristic_map[i][j] !== -Infinity && isPointDefined(i, j))
			{
				heuCal(i, j, heuristic_map[i][j]);
			}
		}
	}

	searchForCurrMinHeuVal();
	goal_val = curr_min_heu_val;
}

this.init = function(c, playerData, selfIdx)
{
	let timer_start = performance.now();

	map = c;
	players = playerData;
	self_idx = selfIdx;
	my_player = players[self_idx];

	heuristic_map = [];
	is_used_rec = [];

	/** 
	 * Initialize the  heuristic and recursive matrix.
	 * Heuristic matrix contains -Infinity values.
	 * Recursive matrix contains false boolean values.
	 * 
	 * In heuristic matrix:
	 * -Infinity -> out of track or wall
	 * value -> on the track
	 * 
	 * During the recursive mapping:
	 * False -> not check yet
	 * True -> already checked
	*/
	for(let i = 0; i < map.length; i++)
	{
		heuristic_map.push([]);
		is_used_rec.push([]);

		for(let j = 0; j < map[i].length; j++)
		{
			heuristic_map[i].push(-Infinity);
			is_used_rec[i].push(false);
		}
	}

	init_pos = 
	{
		x: my_player.pos.x,
		y: my_player.pos.y
	};

	init_state = 
	{
		cs: init_pos,
		ps: init_pos,
		f: 0,
		parent: null
	};

	/**
	 * This time the heuristic value mapping starts from start position and has a high value.
	*/
	let start_heu_val = (map.length * map[0].length);
	heuCal(init_pos.x, init_pos.y, start_heu_val);

	searchForCurrMinHeuVal();
	goal_val = curr_min_heu_val;

	let timer_end = performance.now();
	let run_time = (timer_end - timer_start) / 1000;
	console.log("Init runtime: " + run_time.toFixed(3) + "s");
}

this.moveFunction = function(c, playerData, selfIdx)
{
	let timer_start = performance.now();

	players = playerData;
	self_idx = selfIdx;
	my_player = players[self_idx];

	/**
	 * After each step it updates the map and expand the vision of the map.
	*/
	updateMap(c);

	/** 
	 * After each step it updates the heuristic map and expand the vision of the map.
	*/
	updateHeuMap();

	let curr_pos = 
	{
		x: my_player.pos.x,
		y: my_player.pos.y
	};

	let prev_pos = 
	{
		x: my_player.oldpos.x,
		y: my_player.oldpos.y
	};


	// Properties of state:
	//   cs 	- current state x, y coordinates.
	//   ps		- previous state x, y coordinates.
	//   f      - evaluation value -> f = g + h (g is the weight of the state, h is the heuristic value), 
	//			  at the beginning of moveFunc f is 0.
	// parent   - Previous state, the current state's parent is null.
	let curr_state = 
	{
		cs: curr_pos,
		ps: prev_pos,
		f: 0,
		parent: null
	};

	aStar(curr_state);

	let new_center = 
	{
		x: my_player.pos.x + (my_player.pos.x - my_player.oldpos.x),
		y: my_player.pos.y + (my_player.pos.y - my_player.oldpos.y)
	};

	let moving = 
	{
		x: path[0].cs.x - new_center.x,
		y: path[0].cs.y - new_center.y 
	}

	let timer_end = performance.now();
	let runtime = (timer_end - timer_start) / 1000;

	console.log("moveFunction runtime: " + runtime + "s");

	console.log(self_idx);
	console.log("Stepped to heuristic value: ", heuristic_map[path[0].cs.x][path[0].cs.y]);
	console.log("Current Goal Value: ", goal_val);
	printHeuMap();
	printMap();

	return moving;
}
