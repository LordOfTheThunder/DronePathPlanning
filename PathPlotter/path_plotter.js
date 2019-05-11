var fps, fpsInterval, startTime, now, then, elapsed;
var width, height;
var translated_start_point, translated_path_coordinates, translated_coordinates;
var translated_intersection_polygons = [];
var translated_obstacle_bboxes = [];
var translated_grid = [];
var draw_pos = 0;

var drawn_objects = ["drawIntersections()", 
					 "drawObstacles()",
					 "drawStartPoint()",
					 "drawSensors()"];

function init() {
	// Calculate variables
	calcVariables();
	// Init canvas
	initCanvas();
	// Translate coordinates to our screen sizeToContent
	translateCoordinates();
	// Draw the intersections
	drawIntersections();
	// Draw the obstacles
	drawObstacles();
	// Draw start point - path1.start_point
	drawStartPoint();
	// Draw sensors with radius - path1.sensors
	drawSensors();
	// Draw path animation - path1.path
	startAnimating(fps);
}

function calcVariables() {
	fps = path_points.length / animation_time;
}

function initCanvas() {
	var c = document.getElementById("myCanvas");
	var coords = findLimitCoordinates();
	var rightmost = coords[0], leftmost = coords[1], topmost = coords[2], bottommost = coords[3];
	width_height_ratio = (rightmost - leftmost) / (bottommost - topmost);
	c.height = document.body.clientWidth / 2;
	c.width = c.height * width_height_ratio;
	width = c.width;
	height = c.height;
}

function findLimitCoordinates() {
	var points_list = [start_point].concat(sensor_points);
	var rightmost = 0, leftmost = Infinity, bottommost = 0, topmost = Infinity;
	for (var i = 0; i < points_list.length; i++) {
		var point = points_list[i];
		var x = point[1];
		var y = point[0];
		var radius = point[2];
		rightmost = Math.max(rightmost, x + radius);
		leftmost = Math.min(leftmost, x - radius);
		bottommost = Math.max(bottommost, y + radius);
		topmost = Math.min(topmost, y - radius);
	}
	return [rightmost, leftmost, topmost, bottommost];
}

function translateCoordinates() {
	// start_point holds the start point (2d point)
	// sensor_points hold the locations of the sensors + their radii (3d point)
	var points_list = [start_point].concat(sensor_points);
	
	// path_points hold the locations of the path (2d point)
	var path_list = [start_point].concat(path_points).concat([start_point]);
	
	// Sub rect - smaller than width,height rect
	sub_width = width * 0.8;
	sub_height = height * 0.8;
	
	function translatePointFromLimits(coords, point) {
		var rightmost = coords[0], leftmost = coords[1], topmost = coords[2], bottommost = coords[3];
		var x = point[1];
		var y = point[0];
		var new_x = (x - leftmost) / (rightmost - leftmost) * sub_width + (width - sub_width) / 2;
		var new_y = (y - bottommost) / (topmost - bottommost) * sub_height + (height - sub_height) / 2;
		return [new_x, new_y]
	}
	
	function translateBboxFromLimits(coords, bbox) {
		var bl = bbox[0];
		var ur = bbox[1];
		return [translatePointFromLimits(coords, bl), translatePointFromLimits(coords, ur)]
	}
	
	// For translating start point and coords
	function translateFromLimits(coords, points_list) {
		var rightmost = coords[0], leftmost = coords[1], topmost = coords[2], bottommost = coords[3];
		var translated_points = [];
		for (var i = 0; i < points_list.length; i++) {
			var point = points_list[i];
			var new_point = translatePointFromLimits(coords, point);
			var radius = point[2];
			var new_radius = radius / (rightmost - leftmost) * sub_width;
			translated_points.push([new_point[0], new_point[1], new_radius]);
		}
		return translated_points;
	}
	
	function translateBboxesFromLimits(coords, bboxes) {
		var translated_bboxes = [];
		for (var i = 0; i < bboxes.length; i++) {
			var bbox = bboxes[i];
			var new_bbox = translateBboxFromLimits(coords, bbox);
			translated_bboxes.push(new_bbox);
		}
		return translated_bboxes;
	}
	var coords = findLimitCoordinates();
	// First point returned is the start point
	translated_coordinates = translateFromLimits(coords, points_list);
	// Calculate for path_list
	translated_path_coordinates = translateFromLimits(coords, path_list);
	// Calculate for intersection polygons
	for (var i = 0; i < intersection_point_path.length; i++) {
		translated_intersection_polygons.push(translateFromLimits(coords, intersection_point_path[i]));
	}
	// Calculate for obstacle bboxes
	translated_obstacle_bboxes = translateBboxesFromLimits(coords, obstacle_bboxes);
	// Calculate for grid
	translated_grid = translateBboxesFromLimits(coords, grid_lines);
}

function drawGrid() {
	var c = document.getElementById("myCanvas");
	var ctx = c.getContext("2d");
	
	for (var i = 0; i < translated_grid.length; i++) {
		var curr_line = translated_grid[i];
		var x1 = curr_line[0][0], y1 = curr_line[0][1], x2 = curr_line[1][0], y2 = curr_line[1][1];
		ctx.save();
		
		ctx.beginPath();
		ctx.strokeStyle = "#7D5800";
		ctx.rect(x1, y2, x2 - x1, y1 - y2);
		ctx.stroke();
		
		ctx.restore();
	}
}

function drawObstacles() {
	var c = document.getElementById("myCanvas");
	var ctx = c.getContext("2d");
	
	for (var i = 0; i < translated_obstacle_bboxes.length; i++) {
		var curr_bbox = translated_obstacle_bboxes[i];
		var x1 = curr_bbox[0][0], y1 = curr_bbox[0][1], x2 = curr_bbox[1][0], y2 = curr_bbox[1][1];
		ctx.save();
		
		ctx.beginPath();
		ctx.fillStyle = "#524C8D";
		ctx.rect(x1, y2, x2 - x1, y1 - y2);
		ctx.fill();
		
		ctx.restore();
	}
}

function drawIntersections() {
	var c = document.getElementById("myCanvas");
	var ctx = c.getContext("2d");
	
	for (var i = 0; i < translated_intersection_polygons.length; i++) {
		ctx.save();
		
		ctx.beginPath();
		ctx.fillStyle = "#4B9AFF"
		ctx.moveTo(translated_intersection_polygons[i][0][0], translated_intersection_polygons[i][0][1]);
		for (var j = 1; j < translated_intersection_polygons[i].length; j++) {
			ctx.lineTo(translated_intersection_polygons[i][j][0], translated_intersection_polygons[i][j][1]);
		}
		ctx.moveTo(translated_intersection_polygons[i][0][0], translated_intersection_polygons[i][0][1]);
		ctx.closePath();
		ctx.fill();
		
		ctx.restore();
	}
}

function drawSensors() {
  var c = document.getElementById("myCanvas");
  var ctx = c.getContext("2d");
  
  for (var i = 1; i < translated_coordinates.length; i++) {
	var translated_coordinate = translated_coordinates[i];
	
	ctx.save();
  
	// Draw the sensors
	ctx.beginPath();
	ctx.fillStyle = "#99F929"
	ctx.arc(translated_coordinate[0], translated_coordinate[1], draw_point_width, 0, Math.PI * 2, true);
	ctx.fill();

	ctx.restore();
	
	// Draw sensor radius
	ctx.save();
	
	ctx.beginPath();
	ctx.strokeStyle = "#FF0101"
	ctx.arc(translated_coordinate[0], translated_coordinate[1], translated_coordinate[2], 0, Math.PI * 2, true);
	ctx.stroke();
	
	ctx.restore();
	
	// Draw sensor text
	ctx.save();
  
	ctx.beginPath();
	ctx.font = "bold " + text_size + "px Arial";
	var split_coord = sensor_points[i - 1];
	ctx.fillText("(" + split_coord[1] + "," + split_coord[0] + ")", translated_coordinate[0], translated_coordinate[1]);
	ctx.fillText("r = " + split_coord[2], translated_coordinate[0], translated_coordinate[1] + text_size);
	
	ctx.restore();
  }
}

function drawStartPoint() {
  var c = document.getElementById("myCanvas");
  var ctx = c.getContext("2d");
  
  translated_start_point = translated_coordinates[0];
  // Draw start point
  ctx.save();
  
  ctx.beginPath();
  ctx.fillStyle = "#1034C4"
  ctx.arc(translated_start_point[0], translated_start_point[1], draw_point_width, 0, Math.PI * 2, true);
  ctx.fill();
  
  ctx.restore();
  
  // Draw start point text
  ctx.save();
  
  ctx.beginPath();
  ctx.font = "bold 12px Arial";
  ctx.fillText("(" + start_point[1] + "," + start_point[0] + ")", translated_start_point[0], translated_start_point[1]);
  
  ctx.restore();
}

function drawPath() {
  var c = document.getElementById("myCanvas");
  var ctx = c.getContext("2d");
  
  // Draw path line
  ctx.save();
  
  ctx.beginPath();
  ctx.strokeStyle = "#5F0505"
  var curr_cord = translated_path_coordinates[draw_pos];
  var curr_action = path_actions[draw_pos];
  var next_cord = translated_path_coordinates[++draw_pos];
  var curr_x = curr_cord[0];
  var curr_y = curr_cord[1];
  var next_x = next_cord[0];
  var next_y = next_cord[1];
  ctx.moveTo(curr_x, curr_y);
  ctx.lineTo(next_x, next_y);
  ctx.stroke();
  
  ctx.restore();
  
  if (show_path_ordering) {
	// Draw line ordering
	ctx.save();
	  
	ctx.font = "bold " + path_ordering_text_size + "px Arial";
	ctx.fillText(draw_pos, (curr_x + next_x) / 2, (curr_y + next_y) / 2);
	  
	ctx.restore();
  }
  
  if (curr_action == "stop") {
	// Draw Pause text
	ctx.save();

	ctx.font = "bold " + path_ordering_text_size + "px Arial";
	ctx.fillStyle = "#3be200"
	ctx.fillText("Waiting", next_x, next_y - 20);

	ctx.restore();

	// Draw Pause location
	ctx.save();
	
	ctx.beginPath();
	ctx.fillStyle = "#3B347A"
	ctx.arc(next_x, next_y, draw_point_width, 0, Math.PI * 2, true);
	ctx.fill();
	
	ctx.restore();
  }
}

// initialize the timer variables and start the animation

function startAnimating(fps) {
    fpsInterval = 1000 / fps;
    then = Date.now();
    startTime = then;
    animate();
}

// the animation loop calculates time elapsed since the last loop
// and only draws if your specified fps interval is achieved

function animate() {
	// request another frame

    var myReq = requestAnimationFrame(animate);

    // calc elapsed time since last loop

    now = Date.now();
    elapsed = now - then;

    // if enough time has elapsed, draw the next frame

    if (elapsed > fpsInterval) {

        // Get ready for next frame by setting then=now, but also adjust for your
        // specified fpsInterval not being a multiple of RAF's interval (16.7ms)
        then = now - (elapsed % fpsInterval);

        // Put your drawing code here
		if (draw_pos == translated_path_coordinates.length - 1) {
			cancelAnimationFrame(myReq);
			return;
		}
		
		drawPath();
    }
}

function redraw() {
	// clear canvas
	var c = document.getElementById("myCanvas");
	var ctx = c.getContext("2d");
	ctx.clearRect(0, 0, c.width, c.height);
	for (var i = 0; i < drawn_objects.length; i++) {
		eval(drawn_objects[i]);
	}
	var saved_pos = draw_pos;
	draw_pos = 0;
	for (var i = 0; i < saved_pos; i++) {
		drawPath();
	}
}

function gridOn() {
	if (grid_flag) {
		drawGrid();
		document.getElementById("gridButton").innerText = "Grid On";
	}
	else {
		redraw();
		document.getElementById("gridButton").innerText = "Grid Off";
	}
	grid_flag = !grid_flag;
}

init();