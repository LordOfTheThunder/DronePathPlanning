var fps = 2, fpsInterval, startTime, now, then, elapsed;
var width, height;
var translated_start_point, translated_path_coordinates, translated_coordinates;
var draw_point_width = 4;
var draw_pos = 0;

function init() {
	// Init canvas
	initCanvas();
	// Translate coordinates to our screen sizeToContent
	translateCoordinates();
	// Draw start point - path1.start_point
	drawStartPoint();
	// Draw sensors with radius - path1.sensors
	drawSensors();
	// Draw path animation - path1.path
	startAnimating(fps);
}

function initCanvas() {
	var c = document.getElementById("myCanvas");
	c.width = document.body.clientWidth;
	c.height = document.body.clientHeight;
	width = c.width;
	height = c.height;
}

function translateCoordinates() {
	// start_point holds the start point (2d point)
	// sensor_points hold the locations of the sensors + their radii (3d point)
	var full_points = start_point.concat(" ");
	full_points = full_points.concat(sensor_points);
	points_list = full_points.split(" ");
	
	// path_points hold the locations of the path (2d point)
	var full_path_points = start_point.concat(" ");
	full_path_points = full_path_points.concat(sensor_points);
	full_path_points = full_path_points.concat(" ");
	full_path_points = full_path_points.concat(start_point);
	path_list = full_path_points.split(" ");
	
	function findLimitCoordinates() {
		var rightmost = 0, leftmost = width, bottommost = 0, topmost = height;
		for (var i = 0; i < points_list.length; i++) {
			var point = points_list[i];
			var splitted_point = point.split(",");
			var x = parseFloat(splitted_point[0], 10);
			var y = parseFloat(splitted_point[1], 10);
			var radius = parseFloat(splitted_point[2], 10);
			rightmost = Math.max(rightmost, x + radius);
			leftmost = Math.min(leftmost, x - radius);
			bottommost = Math.max(bottommost, y + radius);
			topmost = Math.min(topmost, y - radius);
		}
		return [rightmost, leftmost, topmost, bottommost];
	}
	
	function translateFromLimits(coords, points_list) {
		var rightmost = coords[0], leftmost = coords[1], topmost = coords[2], bottommost = coords[3];
		var translated_points = [];
		for (var i = 0; i < points_list.length; i++) {
			var point = points_list[i];
			var splitted_point = point.split(",");
			var x = parseFloat(splitted_point[0], 10);
			var y = parseFloat(splitted_point[1], 10);
			// Sub rect - smaller than width,height rect
			sub_width = width * 0.8;
			sub_height = height * 0.8;
			var new_x = (x - leftmost) / (rightmost - leftmost) * sub_width + (width - sub_width) / 2;
			var new_y = (y - bottommost) / (topmost - bottommost) * sub_height + (height - sub_height) / 2;
			if (splitted_point.length == 3) {
				translated_points.push([new_x, new_y, splitted_point[2]]);
			} else {
				translated_points.push([new_x, new_y]);
			}
		}
		return translated_points;
	}
	var coords = findLimitCoordinates();
	// First point returned is the start point
	translated_coordinates = translateFromLimits(coords, points_list);
	// Calculate for path_list
	translated_path_coordinates = translateFromLimits(coords, path_list);
}

function drawSensors() {
  var c = document.getElementById("myCanvas");
  var ctx = c.getContext("2d");
  
  sensor_point_list = sensor_points.split(" ");
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
	ctx.font = "bold 12px Arial";
	var split_coord = sensor_point_list[i - 1].split(",");
	ctx.fillText("(" + split_coord[0] + "," + split_coord[1] + ")", translated_coordinate[0], translated_coordinate[1]);
	  
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
  var split_start_point = start_point.split(",");
  ctx.fillText("(" + split_start_point[0] + "," + split_start_point[1] + ")", translated_start_point[0], translated_start_point[1]);
  
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
  var next_cord = translated_path_coordinates[++draw_pos];
  var curr_x = curr_cord[0];
  var curr_y = curr_cord[1];
  var next_x = next_cord[0];
  var next_y = next_cord[1];
  ctx.moveTo(curr_x, curr_y);
  ctx.lineTo(next_x, next_y);
  ctx.stroke();
  
  ctx.restore();
  
  // Draw line ordering
  ctx.save();
  
  ctx.font = "bold 16px Arial";
  ctx.fillText(draw_pos, (curr_x + next_x) / 2, (curr_y + next_y) / 2);
  
  ctx.restore();
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

init();