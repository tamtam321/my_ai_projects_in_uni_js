typeof Infinity;

let map = [];
let heuristic_map = [];
let players;
let self_idx;
let is_used_rec = []	// Matrix where each cell tells you which one has been already used for recursive call.

let path = [];	// Path from current state to goal state.
let step_history = [];	// Step history

// Print heuristic map
function printHeuMap()
{
	let str = "";

	for(let i = 0; i < heuristic_map.length; i++)
	{
		for(let j = 0; j < heuristic_map[0].length; j++)
		{
			if(heuristic_map[i][j] === Infinity)
			{
				str += "x   ";
			}
			else if(heuristic_map[i][j] >= 0 && heuristic_map[i][j] <= 9)
			{
				str += heuristic_map[i][j] + "   ";
			}
			else if(heuristic_map[i][j] >= 10 && heuristic_map[i][j] <= 99)
			{
				str += heuristic_map[i][j] + "  ";
			}
			else
			{
				str += heuristic_map[i][j] + " ";
			}
		}

		str += "\n";
	}

	console.log(str);
	console.log();
}

// Print map
function printMap()
{
	let str = "";

	for(let i = 0; i < map.length; i++)
	{
		for(let j = 0; j < map[0].length; j++)
		{
			if(map[i][j] === -1)
			{
				str += map[i][j] + "   ";
			}
			else if(map[i][j] >= 0 && map[i][j] <= 9)
			{
				str += " "+ map[i][j] + "   ";
			}
			else if(map[i][j] >= 10 && map[i][j] <= 99)
			{
				str += " "+ map[i][j] + "  ";
			}
			else
			{
				str += " "+ map[i][j] + " ";
			}
		}

		str += "\n";
	}

	console.log(str);
	console.log();
}

// Check if the agent is on the track.
function onTrack(x, y)
{
	return x >= 0 && y >= 0 && x < map.length && y < map[0].length && map[x][y] >= 0;
}

// Check if the agent is in the map.
function inMap(x, y)
{
	return x >= 0 && y >= 0 && x < map.length && y < map[0].length;
}

// Heuristic value calculator (recursive method)
function heuCal(x, y, hvalue)
{
	if(!is_used_rec[x][y])
	{
		heuristic_map[x][y] = hvalue;
		is_used_rec[x][y] = true;
		let valid_steps = [];	// Stores cells which haven't been used yet for calculating its neighbours' heuristic value.

		// Checking current cell's neighbours
		// and calculate theirs heuristic value.
		for(let i = -1; i <= 1; i++)
		{
			for(let j = -1; j <= 1; j++)
			{
				if(!(i === 0 && j === 0))	// Don't need to consider the current cell, because it has been already checked.
				{
					let step_x = x + i;
					let step_y = y + j;

					if(onTrack(step_x, step_y))
					{
						if(map[step_x][step_y] === 100)
						{
							heuristic_map[step_x][step_y] = 0;
							valid_steps.push([step_x, step_y, 0]);
						}
						else if(hvalue + 1 < heuristic_map[step_x][step_y])
						{
							heuristic_map[step_x][step_y] = hvalue + 1;
							valid_steps.push([step_x, step_y, hvalue + 1]);
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

function startHeuCal()	// Finding the goal and start the heuristic calculation from there.
{
	for(let i = 0; i < map.length; i++)
	{
		for(let j = 0; j < map[i].length; j++)
		{
			if(map[i][j] === 100)	// Here we got one of the goal cell, because there is cells area of goal
			{
				heuCal(i, j, 0);
				return;
			}
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

function isGoal(state)
{
	return map[state.cs.x][state.cs.y] === 100;
}

function mapNeighbours(state)
{
	let neighbour_states = [];	// List of cells where the agent could step.
	let other_choices = [];	// If there is no good choices to make, then it chooses other options which are collide with wall or other player.

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

			// If next step is valid and stays on the track.
			if(lc.validLine(state.cs, next_step) && inMap(next_step.x, next_step.y) && heuristic_map[next_step.x][next_step.y] !== Infinity)
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
				if((lc.playerAt(next_step) < 0 || lc.playerAt(next_step) === self_idx) && map[next_step.x][next_step.y] >= 0)
				{
					neighbour_states.push(new_state);	// Stores valid steps.
				}
				else
				{
					other_choices.push(new_state);	// Stores invalid steps.
				}
			}
		}
	}

	// If it is unable to make valid move then chooses invalid move.
	if(neighbour_states.length === 0)
	{
		for(let i = 0; i < other_choices.length; i++)
		{
			neighbour_states.push(other_choices[i]);
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
	let possible_steps = [];	// list of possible steps
	possible_steps.push(state);
	let goal_state = null;	// Here I'm gonna store the goal state.

	/**
	 * If there is good steps then it is gonna move there until it reaches the goal.
	 * Check every possible steps and consider those which are valid and not gonna lead the
	 * agent into the wall or other player. When it finds the goal, then from there it builds
	 * up the path by propagating back from its parent up to the current state.
	 * Each time when they call the moveFunc it is gonna build up the path again, because
	 * the state of the track is gonna change and it has to calculate everything again.
	 * 
	 * Running time constraint -> 900ms
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

		possible_steps = sortList(possible_steps);	// Sort by heuristic value, it is gonna take the better one first.

		let timer_end = performance.now();
		timer_sum = timer_end - timer_start;
	}

	backpropagation(goal_state);	// Got the goal and build up the path.

	return goal_state;
}

this.init = function(c, playerData, selfIdx)
{
	let timer_start = performance.now();

	map = c;
	players = playerData;
	self_idx = selfIdx;

	heuristic_map = [];
	is_used_rec = [];

	/** 
	 * Initialize the  heuristic and recursive matrix.
	 * Heuristic matrix contains Infinity values.
	 * Recursive matrix contains false boolean values.
	 * 
	 * In heuristic matrix:
	 * Infinity -> out of track or wall
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
			heuristic_map[i].push(Infinity);
			is_used_rec[i].push(false);
		}
	}

	startHeuCal();

	let timer_end = performance.now();
	let run_time = (timer_end - timer_start) / 1000;
	console.log("Init runtime: " + run_time.toFixed(3) + "s");

	printHeuMap();
	printMap()
}

this.moveFunction = function(c, playerData, selfIdx)
{
	let timer_start = performance.now();

	players = playerData;
	self_idx = selfIdx;
	let my_player = playerData[selfIdx];

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

	printHeuMap();
	printMap();

	return moving;
}
