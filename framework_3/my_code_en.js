typeof Infinity;
typeof -Infinity;

let map = [];
let players;
let my_player;
let self_idx;
let init_pos;
let init_state;
let goal_val = Infinity;

let curr_min_heu_val = Infinity;	// Tracking the current smallest heuristic value to know when it reaches the goal.
let curr_min_heu_val_x = 0;	// Current smallest heuristic value x coordinate.
let curr_min_heu_val_y = 0;	// Current smallest heuristic value y coordinate.

let path = [];	// Path from current state to goal state.
let step_history = [];	// Step history

let heuristic_map = [];
let is_used_rec = [];	// Matrix where each cell tells you which one has been already used for recursive call.

let heuristic_map_for_traps = []; // Heuristic map, where sand and oil traps are considered.
let is_used_rec2 = [];	// Recursive boolean matrix for heuristic_map_for_traps matrix. (like is_used_rec)

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

// Print heuristic_map_for_traps
function printHeuMapTraps()
{
	let str = "";
	let map_length = heuristic_map_for_traps.length * heuristic_map_for_traps[0].length;
	let tmp = "";

	for(let i = 0; i < heuristic_map_for_traps.length; i++)
	{
		for(let j = 0; j < heuristic_map_for_traps[0].length; j++)
		{
			if(heuristic_map_for_traps[i][j] === Infinity)
			{
				tmp = "x";
			}
			else
			{
				tmp = heuristic_map_for_traps[i][j].toString();
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

					// If the neighbour is valid and on the track.
					if(isPointDefined(step_x, step_y) && onTrack(step_x, step_y))
					{
						if(map[step_x][step_y] === 100)
						{
							heuristic_map[step_x][step_y] = 0;
							valid_steps.push([step_x, step_y, 0]);
						}
						else if(hvalue - 1 > heuristic_map[step_x][step_y])	// Start from the beginning of the track and with each step the heuristic value decrease.
						{
							heuristic_map[step_x][step_y] = hvalue - 1;
							valid_steps.push([step_x, step_y, hvalue - 1]);
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
	return heuristic_map_for_traps[state.cs.x][state.cs.y];
}

/**
 * Because it couldn't see the whole map at the beginning, so it is gonna use the 
 * heuristic matrix to estimate the goal direction until it reaches the goal.
 * The heuristic value of the goal is 0.
*/
function isGoal(state)
{
	return heuristic_map_for_traps[state.cs.x][state.cs.y] === goal_val;
}

function distanceSort(list)
{
	return list.sort(function(a, b)
	{
		if(a.distance < b.distance)
		{
			return -1;
		}
		if(a.distance > b.distance)
		{
			return 1;
		}
		return 0;
	});
}

function mapNeighbours(state, step_counter)
{
	let neighbour_states = [];	// List of cells where the agent could step.

	// If there is no good choices to make, then it chooses other options which are collide with wall or other player.
	let other_choices = [];

	let newCenter = 
	{
		x: state.cs.x + (state.cs.x - state.ps.x),
		y: state.cs.y + (state.cs.y - state.ps.y)
	}

	let map_value = map[state.cs.x][state.cs.y];

	// Stepped on oil or sand trap and the velocity is not null.
	if((map_value === 91 || map_value ===92) && (state.cs.x !== state.ps.x || state.cs.y !== state.ps.y))
	{
		/**
		 * In the task description it tells how the traps distract the agent and we can estimate the possible
		 * landing position. With this we can check if there is a danger factor of the distraction.
		*/
		let possible_outcome = [];

		for(let i = -1; i <= 1; i++)
		{
			for(let j = -1; j <= 1; j++)
			{
				let next_step = {x: newCenter.x + i, y: newCenter.y + j};
				let distance = 0;

				// Stepped on oil -> distance between new center and next possible steps.
				if(map_value === 91)
				{
					let distance_center = Math.sqrt(Math.pow(state.cs.x - newCenter.x, 2) + Math.pow(state.cs.y - newCenter.y, 2));
					let distance_step = Math.sqrt(Math.pow(state.cs.x - next_step.x, 2) + Math.pow(state.cs.y - next_step.y, 2));
					distance = Math.abs(distance_step - distance_center);
				}
				else if(map_value === 92)	// Stepped on sand -> distance of next steps.
				{
					distance = Math.sqrt(Math.pow(state.cs.x - next_step.x, 2) + Math.pow(state.cs.y - next_step.y, 2));
				}

				next_step.distance = distance;
				possible_outcome.push(next_step);
			}
		}

		/**
		 * Stepped on sand -> Calculate the three smallest distances of the distraction and 
		 * those positions are where it could distract the agent. 
		 * Examine them which one has danger factor.
		 * 
		 * Stepped on sand -> Calculate the distances of the distraction and 
		 * Examine them which one is the closest to the new center and that's gonna be where it could distract the agent.
		 * And check if it has a danger factor.
		*/
		distanceSort(possible_outcome);	// I will sort them in ascending order by distance.
		let step_weight = 1;	// If there is a wall collision among the possible steps, the weight of the step increases.
		for(let i = 0; i < 3; i++)	// Determine step weight.
		{
			let next_step = {x: possible_outcome[i].x, y: possible_outcome[i].y};
			
			// If there is a collision, then the weight of the step is gonna increase.
			if(!(lc.validVisibleLine(map, state.cs, next_step) && heuristic_map_for_traps[next_step.x][next_step.y] !== Infinity))
			{
				step_weight += 10;
			}
		}

		for(let i = 0; i < 3; i++)
		{
			let next_step = {x: possible_outcome[i].x, y: possible_outcome[i].y};

			// Steps that do not lead to an invalid step.
			if(lc.validVisibleLine(map, state.cs, next_step) && heuristic_map_for_traps[next_step.x][next_step.y] !== Infinity)
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
					w: step_weight
				};

				/**
				 * During mapping the possible outcome, only at the first step it checks if there is another player.
				 * After that it doesn't have to anymore, because those steps are just estimation and the other player
				 * maybe will not be there. And still try to avoid wall collision.
				*/
				if(step_counter === 1)
				{
					if((lc.visiblePlayerAt(map, next_step) < 0 || lc.visiblePlayerAt(map, next_step) === self_idx))
					{
						neighbour_states.push(new_state);	// Stores valid steps.
					}
					else
					{
						other_choices.push(new_state);	// Stores invalid steps.
					}
				}
				else	// Apart from the first step, the rest is foresight and it doesn't matter if there is a player there or not, the main thing is not to hit a wall.
				{
					neighbour_states.push(new_state);
				}
			}
		}
	}
	else	// not stepped on oil or sand or velocity is null
	{
		for(let i = -1; i <= 1; i++)
		{
			for(let j = -1; j <= 1; j++)
			{
				let next_step = {x: newCenter.x + i, y: newCenter.y + j};

				/**
				 * If next step is valid and stays on the track.
				*/
				if(lc.validVisibleLine(map, state.cs, next_step) && heuristic_map_for_traps[next_step.x][next_step.y] !== Infinity)
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
					
					/**
					 * During mapping the possible outcome, only at the first step it checks if there is another player.
					 * After that it doesn't have to anymore, because those steps are just estimation and the other player
					 * maybe will not be there. And still try to avoid wall collision.
					*/
					if(step_counter === 1)
					{
						if((lc.visiblePlayerAt(map, next_step) < 0 || lc.visiblePlayerAt(map, next_step) === self_idx))
						{
							neighbour_states.push(new_state);	// Stores valid steps.
						}
						else
						{
							other_choices.push(new_state);	// Stores invalid steps.
						}
					}
					else	// Apart from the first step, the rest is foresight and it doesn't matter if there is a player there or not, the main thing is not to hit a wall.
					{
						neighbour_states.push(new_state);
					}
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
	let possible_steps = [];	// list of possible steps.
	possible_steps.push(state);
	let goal_state = null;	// Here I'm gonna store the goal state.
	let step_counter = 0;

	/**
	 * If there is valid step then it is gonna check it, until it reaches in the current vision the smallest
	 * heuristic value which stands next to an undefined point. At the beginning it doesn't know where the goal is.
	 * With each time heading to the smallest heuristic value it is gonna reach the smallest value which is 0 and it is 
	 * the goal. The runtime of the function is 900ms, so the agent has limited time to think.
	 * 
	 * This is the possible path search.
	*/
	while(possible_steps.length > 0 && timer_sum < 900)
	{
		let curr_state = possible_steps.shift();
		step_history.push(curr_state);
		goal_state = curr_state;
		step_counter++;

		if(isGoal(curr_state))
		{
			break;
		}

		let neighbour_states = mapNeighbours(curr_state, step_counter);

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

/**
 * Does it has undefined neighbour?
 * This is needed to look for the smallest
 * heuristic value.
*/
function hasUndefinedNeighbour(x, y)
{
	for(let i = -1; i <= 1; i++ )
	{
		for(let j = -1; j <= 1; j++)
		{
			if(i !== 0 || j !== 0)
			{
				let step_x = x + i;
				let step_y = y + j;

				if(inMap(step_x, step_y) && !isPointDefined(step_x, step_y))
				{
					return true;
				}
			}
		}
	}

	return false;
}

/**
 * Search for the smallest heuristic value on the track, which is
 * next to an undefined area. If it goes that way, than it ables to explore the map and
 * find the right direction to the goal.
*/
function searchForCurrMinHeuVal()
{
	curr_min_heu_val = Infinity;

	for(let i = 0; i < heuristic_map.length; i++)
	{
		for(let j = 0; j < heuristic_map[0].length; j++)
		{
			/**
			 * The agent keeps looking for the right direction based on the smallest
			 * heuristic value of the current vision and when the goal is gets in the vision,
			 * then it is able to build up a path.
			*/
			if(heuristic_map[i][j] === 0)
			{
				curr_min_heu_val = heuristic_map[i][j];
				curr_min_heu_val_x = i;
				curr_min_heu_val_y = j;
				
				return;
			}
			else if(heuristic_map[i][j] !== -Infinity && heuristic_map[i][j] < curr_min_heu_val && hasUndefinedNeighbour(i,j))
			{
				curr_min_heu_val = heuristic_map[i][j];
				curr_min_heu_val_x = i;
				curr_min_heu_val_y = j;
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
 * From the current smallest heuristic value it starts calculating heuristic backward to
 * the beginning of the map. It adjusts the heuristic values.
 * 
*/
function heuCalTraps(x, y, hvalue)
{
	if(!is_used_rec2[x][y])
	{
		heuristic_map_for_traps[x][y] = hvalue;
		is_used_rec2[x][y] = true;
		let valid_steps = [];	// Stores cells which haven't been used yet for calculating its neighbours' heuristic value.

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
							heuristic_map_for_traps[step_x][step_y] = 0;
							valid_steps.push([step_x, step_y, 0]);
						}
						else if(hvalue + 1 < heuristic_map_for_traps[step_x][step_y])
						{
							heuristic_map_for_traps[step_x][step_y] = hvalue + 1;
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
			heuCalTraps(step[0], step[1], step[2]);
		}
	}
}

/**
 * The track has branching route and because of that the heuristic value calculation is also gonna
 * spread in different direction. Some values are gonna be differ from another, which may cause some issue
 * for the agent. That's why I tried to adjust the values by recalculate them.
 * 
 * The traps values are exceptions, those are bigger for avoid purposes.
*/
function adjustHeuMapTraps()
{
	for(let i = 0; i < heuristic_map_for_traps.length; i++)
	{
		for(let j = 0; j < heuristic_map_for_traps[0].length; j++)
		{
			for(let dir_x = -1; dir_x <= 1; dir_x++)
			{
				for(let dir_y = -1; dir_y <= 1; dir_y++)
				{
					if(dir_x !== 0 || dir_y !== 0)
					{
						let step_x = i + dir_x;
						let step_y = j + dir_y;

						if(inMap(step_x, step_y) && heuristic_map_for_traps[step_x][step_y] !== Infinity && 
						heuristic_map_for_traps[step_x][step_y] > heuristic_map_for_traps[i][j] + 1)
						{
							heuristic_map_for_traps[step_x][step_y] = heuristic_map_for_traps[i][j] + 1;
						}
					}
				}
			}
		}
	}
}

/**
 * Pay more attention on sand traps, because it slows the agent down. In heuristic_map_for_traps
 * matrix I increase their values.
*/
function trapsConsidering()
{
	for(let i = 0; i < heuristic_map_for_traps.length; i++)
	{
		for(let j = 0; j < heuristic_map_for_traps[0].length; j++)
		{
			if(map[i][j] === 91 || map[i][j] === 92)	// 91 -> oil, 92 -> sand (value of traps in the map)
			{
				let danger_val = 1;	// increase the traps value this much

				if(map[i][j] === 92)	// Sand trap is worse
				{
					danger_val += 1;
				}

				for(let dir_x = -1; dir_x <= 1; dir_x++)
				{
					for(let dir_y = -1; dir_y <= 1; dir_y++)
					{
						if(dir_x !== 0 || dir_y !== 0)
						{
							let step_x = i + dir_x;
							let step_y = j + dir_y;

							if(heuristic_map_for_traps[i][j] !== Infinity && inMap(step_x, step_y) && 
							isPointDefined(step_x, step_y))
							{
								/**
								 * If there is a wall next to the trap, then the danger factor of the trap is gonna increase.
								 * If the agent tries to avoid the trap it may hit the wall.
								*/
								if(map[step_x][step_y] < 0)
								{
									danger_val += 2;
								}
								else if(map[step_x][step_y] === 92 || map[step_x][step_y] === 91)	// If near the trap there are more traps then also increase the danger val by one.
								{																    // Unlike the wall if avoid a trap leads to step on another trap is better than hitting a wall,
									danger_val += 1;											    // it doesn't got penalty, so I increase the danger val by one.
								}
							}
						}
					}
				}

				if(i !== curr_min_heu_val_x || j !== curr_min_heu_val_y)
				{
					heuristic_map_for_traps[i][j] += danger_val;
				}
			}
		}
	}
}

/** 
 * At the edge of the vision it searches for the smallest heuristic value of the current vision and from there it 
 * starts a heuristic calculation. Because there are branching routes and the calcualtion also spreading it could cause 
 * some problem for the agent when it starts to explore the map and may go around in the same circle route. I tried to
 * adjust the value. Among normal cells they just have one value difference, except traps it could have bigger difference.
 * 
 * In heuristic_map_for_traps matrix I marked the traps.
*/
function heuMapTraps()
{
	heuristic_map_for_traps = [];
	is_used_rec2 = [];

	for(let i = 0; i < map.length; i++)
	{
		heuristic_map_for_traps.push([]);
		is_used_rec2.push([]);

		for(let j = 0; j < map[0].length; j++)
		{
			heuristic_map_for_traps[i].push(Infinity);
			is_used_rec2[i].push(false);
		}
	}

	heuCalTraps(curr_min_heu_val_x, curr_min_heu_val_y, curr_min_heu_val); // calculate heuristic value
	adjustHeuMapTraps();	// for adjust heuristic value
	trapsConsidering();	// mark traps
}

function hasDefinedNeighbour(x, y)
{
	for(let i = -1; i <= 1; i++)
	{
		for(let j = -1; j <= 1; j++)
		{
			if(i !== 0 || j !== 0)
			{
				let step_x = x + i;
				let step_y = y + j;

				if(isPointDefined(x, y) && inMap(step_x, step_y) && map[step_x][step_y] >= 0 && heuristic_map[step_x][step_y] !== -Infinity)
				{
					return true;
				}
			}
		}
	}

	return false;
}

/**
 * Update heuristic matrix.
*/
function updateHeuMap()
{
	if(curr_min_heu_val !== 0)
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
				if(heuristic_map[i][j] !== -Infinity && hasDefinedNeighbour(i, j))
				{
					heuCal(i, j, heuristic_map[i][j]);
				}
			}
		}

		searchForCurrMinHeuVal();
		goal_val = curr_min_heu_val;
		heuMapTraps();
	}
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
	heuMapTraps();

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

	let moving;
	
	if(path.length > 0)
	{
		moving = 
		{
			x: path[0].cs.x - new_center.x,
			y: path[0].cs.y - new_center.y 
		};
	}
	else	// If there is no valid steps, then move this.
	{
		moving = {x: 0,	y: 0};
	}

	let timer_end = performance.now();
	let runtime = (timer_end - timer_start) / 1000;

	//______________________________Printing to the console______________________________
	// //moveFunction runtime
	// console.log("moveFunction runtime: " + runtime + "s");

	// //print the index of the player
	// console.log(self_idx);

	// //Which heuristic value the agent stepped on.
	// console.log("Stepped to heuristic value: ", heuristic_map_for_traps[path[0].cs.x][path[0].cs.y]);

	// //Current smallest heuristic value in the vision.
	// console.log("Current Goal Value: ", goal_val);

	// printHeuMap();
	// printHeuMapTraps();
	// printMap();
	//_________________________________________________________________________________

	return moving;
}
