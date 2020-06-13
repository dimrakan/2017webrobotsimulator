

function getParameterByName(name, url) {
    if (!url) url = window.location.href;
    name = name.replace(/[\[\]]/g, "\\$&");
    var regex = new RegExp("[?&]" + name + "(=([^&#]*)|&|#|$)"),
        results = regex.exec(url);
    if (!results) return null;
    if (!results[2]) return '';
    return decodeURIComponent(results[2].replace(/\+/g, " "));
}



var token=getParameterByName('token');

var pod=1;

function startpack(username,pod){
  if (launchdrop.value!=""){
    document.getElementById("loading").style.display="block"; 
    document.getElementById("lgz").disabled = true; 
    var myfile=new Object();
    myfile.number=launchdrop.value;
    myfile.map="-1";
    $.ajax({
        type: "POST",
        data:JSON.stringify(myfile),
        url: "/users/"+username+"/pods/"+pod+"/sim?token="+token,
        contentType: "application/json",
        beforeSend: function(request) {
               },     
        success:function(data){
         document.getElementById("loading").style.display="none";
         document.getElementById("lgz").disabled = false;
         window.open(data);
      } 
  });
  }
  
  else {
        alert("Upload and choose a launch file first");
  }

}


function build(username,pod){
document.getElementById("loading").style.display="block";
 $.ajax({
        type: "PUT",
        url: "/users/"+username+"/pods/"+pod+"?token="+token,
        contentType: "application/json",
        beforeSend: function(request) {
         document.getElementById("loading").style.display="block";
         document.getElementById("sub").disabled = true;
               },

        success:function(response){
                out=response.pop();
                msg=response.pop();
                addInput('launchdrop','dynamicInput',response,msg);
                document.getElementById("buildout").innerText=out;
                document.getElementById("loading").style.display="none";
                document.getElementById("lgz").disabled = false;
                document.getElementById("sub").disabled = false;
                document.getElementById("loading").style.display="none";
                document.getElementById('dynamicInput').style.display="block";
                
                },
        error: function(jqXHR, textStatus, errorThrown) {
                alert(JSON.parse(jqXHR.responseText));
                document.getElementById("loading").style.display="none";                
            }
   });  

}
function show(username){
    document.getElementById("mit").disabled = false;
    $.ajax({
        type: "DELETE",
        url: "/users/"+username+"/sim_package?token="+token,
        contentType: "application/json",
        success:function(response){
                //alert(response);
                },
        error: function(jqXHR, textStatus, errorThrown) {
                alert(JSON.parse(jqXHR.responseText));
                
            }
   });  
   }




$(window).load(function() {
        $('#termload').hide();
    });

function robotbtn1(username,pods) {
 if (count==number.value) { 
  var robotbody=new Object();
  robotbody.number=myArray[1];
  robotbody.map=myArray[0];
  var arrayrobot=new Array();
  var robot=new Object();
  for (i=0 ; i<count ;i++){
   robot.robotname=myArray[i*9+4];
   robot.base=myArray[i*9+5];
   robot.a3d_sensor=myArray[i*9+6];
   robot.lidar_sensor=myArray[i*9+7]
   robot.pose_x=myArray[i*9+8];
   robot.pose_y=myArray[i*9+9];
   robot.pose_z=myArray[i*9+10];
   arrayrobot.push(robot);
   robot={};}
  console.log(arrayrobot);
  robotbody.robots=arrayrobot;
   $.ajax({
        type: "POST",
        data :JSON.stringify(robotbody),
        url: "/users/"+username+"/pods/"+pods+"/sim?token="+token,
        contentType: "application/json",
        beforeSend: function(request) {
               document.getElementById('robotbtn').disabled=true;
               setTimeout(function(){document.getElementById("robotbtn").disabled=false;},5000);
                 },     
        success:function(response){
                window.open(response);
                },
        error: function(jqXHR, textStatus, errorThrown) {
                alert(JSON.parse(jqXHR.responseText));
                
            }
   });
  }
  else {alert("Not enough robots specified");}
};

$('#gzweb').on('click', function() {
 
               
});

var myArray=new Array();
var count=0;

function addbtn1() {
    count=count+1
    //alert(count); 
    if (count==1) {
      myArray[0]=map.value;
      myArray.push(number.value);
      myArray.push("/");
      //alert(myArray);
    }
    if (count<=number.value) {
      for (i=0;i<count;i++){
       if (document.getElementById("robotname").value==myArray[i*9+4]){
        alert('A robot with the same name already exists');
        count=count-1;
        return;
       }
       else if(document.getElementById("pose_x").value==myArray[i*9+8] && document.getElementById("pose_y").value==myArray[i*9+9] &&          document.getElementById("pose_z").value==myArray[i*9+10]){ 
        alert('A robot in the same position already exists');
        count=count-1;
        return ;
       }}
      myArray[1]=number.value;   
      myArray.push(count);
      myArray.push(document.getElementById("robotname").value);
      myArray.push(document.getElementById("base").value);
      myArray.push(document.getElementById("a3d_sensor").value);
      myArray.push(document.getElementById("lidar_sensor").value);
      myArray.push(document.getElementById("pose_x").value); 
      myArray.push(document.getElementById("pose_y").value); 
      myArray.push(document.getElementById("pose_z").value);
      myArray.push("/");
      var table = document.getElementById("table1");
      var row = table1.insertRow(count);
      var cell1 = row.insertCell(0);
      var cell2 = row.insertCell(1);
      var cell3 = row.insertCell(2);
      var cell4 = row.insertCell(3);
      var cell5 = row.insertCell(4);
      var cell6 = row.insertCell(5);
      var cell7 = row.insertCell(6); 
      var i=count-1;
      cell1.innerHTML = myArray[i*9+4];
      cell2.innerHTML = myArray[i*9+5];
      cell3.innerHTML = myArray[i*9+6];
      cell4.innerHTML = myArray[i*9+7];
      cell5.innerHTML = myArray[i*9+8];
      cell6.innerHTML = myArray[i*9+9];
      cell7.innerHTML = myArray[i*9+10];

      
      
      if (count==number.value) {
        $('#rb_vls').hide();
      }      
    }
    else {
      alert("You have already choose all robots!!!Change number value to add more");
      count=count-1;
    }

}

function change_base(){
  if (base.value=='create'){
    document.getElementById('astra_sensor').disabled=true;
  }
  else {
    document.getElementById('astra_sensor').disabled=false;
   }
  };


function rmbtn1() {
    $('#rb_vls').show();
   
    if (count>1){
      document.getElementById("table1").deleteRow(count);
      count=count-1;
      myArray.splice(-9,9);
      //alert(myArray);
    }
    else if (count==1) {
     myArray.splice(0,12);
     document.getElementById("table1").deleteRow(count); 
     count=count-1;
     //alert(myArray);
    }
    else {
     alert("No robot to remove");
    }
     
};



function resetworld1() {
   myArray=[];
   
   for (i=count;i>=1;i--){
    document.getElementById("table1").deleteRow(i);
    
   }
   count=0;  
   $('#rb_vls').show(); 
  
};

$('#number').change(function(){
   
   if (count<=number.value){
      $('#rb_vls').show();
      document.getElementById('lb_rb_nm').innerHTML ="Robotname"+(count+1);
   }
   if (count>number.value){
      number.value=count;
      alert("remove robots first");
   }  
          
});

$('#map').change(function(){
   
   myArray[0]=map.value;
          
});

 
function selpod(){
 $.ajax({
        type: "GET",
        url: "/users/"+username+"/pods/"+upload_pods.value+"/deployment?token="+token,
        contentType: "application/json",
        beforeSend: function(request) {
         document.getElementById("loading").style.display="block";
         document.getElementById("sub").disabled = true;
               },

        success:function(response){
                out=response.pop();
                console.log(response);
                msg=''
                document.getElementById("lgz").disabled = true;
                document.getElementById("buildout").innerText=out;
                if (response[0]!=null){
                 addInput('launchdrop','dynamicInput',response,msg);                
                 document.getElementById("lgz").disabled = false;
                 document.getElementById('dynamicInput').style.display="block";

                 }
                else  {document.getElementById('dynamicInput').style.display="none";}    
                document.getElementById("loading").style.display="none";
                document.getElementById("sub").disabled = false;
                document.getElementById("loading").style.display="none";
                document.getElementById('upload-form').style.display="block";                 
                },
        error: function(jqXHR, textStatus, errorThrown) {
                alert(JSON.parse(jqXHR.responseText));
                document.getElementById("sub").disabled = true;
                document.getElementById("loading").style.display="none";
                document.getElementById('upload-form').style.display="none";
            }
   });

 
}

function sel2pod(){
 document.getElementById('custom-form').style.display="block";
}

function add_pod(username,pod){
  var pod_psw = prompt("Please set a password for your simulation:");
  if (pod_psw == null || pod_psw == "") {
        alert("Invalid password,pod is has not been created")
  }
  else {
   var pod_pass = {password:pod_psw};
   $.ajax({
        type: "POST",
        data :JSON.stringify(pod_pass),
        url: "/users/"+username+"/pods/"+pod+"?token="+token,
        contentType: "application/json",
        success:function(response){
                document.getElementById('start_pod').innerText='Starting Simulation Environment,Check status for info'  
                },
        error: function(jqXHR, textStatus, errorThrown) {
               // alert(JSON.parse(jqXHR.responseText));
               alert('Error,Try again!') 
            }
   });
}

}

function rm_pod(username,pod){
$.ajax({
        type: "DELETE",
        url: "/users/"+username+"/pods/"+pod+"?token="+token,
        contentType: "application/json",
        success:function(response){
                //window.location.href=response;
                document.getElementById('start_pod').innerText='Deleting Simulation Environment,Check status for info'
                },
        error: function(jqXHR, textStatus, errorThrown) {
                //alert(JSON.parse(jqXHR.responseText));
                document.getElementById('start_pod').InnerText='Error,Try again!'
                
            }
   });  
};

function getTopics(username,pod){
  var table = document.getElementById("TopicsTable");
  document.getElementById("loading").style.display="block";
  document.getElementById("rosbridge").style.display="none";
  document.getElementById("webvideoserver").style.display="none";
  nrows=table.rows.length
  for (i=0;i<nrows;i++){
    table.deleteRow(0);
  };
  $.ajax({
     type:"GET",
     url:"/users/"+username+"/pods/"+pod+"/deployment?token="+token,
     contentType: "application/json",
     success:function(response){
      gz_port=response.pop();
      if (response[0]=="ERROR:"){
       alert('Ros master is not running');}
      else{
       var header = table.createTHead();
       var row = header.insertRow(0);
       var cell = row.insertCell(0);
       cell.innerHTML = "<b>Ros Topics</b>"; 
       for (i=0;i<response.length;i++){
         row = table.insertRow(i+1);
         cell1 = row.insertCell(0);
         cell1.innerHTML = response[i];
        }
      }
     document.getElementById("loading").style.display="none";
     document.getElementById("rosbridge").innerHTML='Rosbridge Port : '+(parseInt(gz_port,10)+2)
     document.getElementById("rosbridge").style.display="block";
     document.getElementById("webvideoserver").innerHTML='Web Video Server Port : '+(parseInt(gz_port,10)+3)
     document.getElementById("webvideoserver").style.display="block";

     },
     error: function(jqXHR, textStatus, errorThrown) {
       alert(JSON.parse(jqXHR.responseText));
       //alert('Cannot get topics,propably simulation environment is not running'); 
       document.getElementById("loading").style.display="none";

     }
     });
};



var hostname=window.location.hostname;





function executeQuery() {
  $.ajax({
    type: "PUT",
    url: "/users/"+username+"/time?token="+token,
    success: function(data) {
       },
    complete: function() {
    setTimeout(executeQuery, 140000);
    }
    
  });   
}

$(document).ready(function() {
  // run the first time; all subsequent calls will take care of themselves
  setTimeout(executeQuery, 140);

  
  //setTimeout(checkstatus, 100);
  
 });


