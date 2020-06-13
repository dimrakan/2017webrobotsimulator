
function login(){
 var usn = document.getElementById("login-username").value;
 var psw = document.getElementById("login-password").value;
 var user = {username :usn,password:psw};
 $.ajax({
        type: "POST",
        data :JSON.stringify(user),
        url: "/users/login",
        contentType: "application/json",
        success:function(response){
                window.location.href=response;
                },
        error: function(jqXHR, textStatus, errorThrown) {
                alert(JSON.parse(jqXHR.responseText));
                
            }
   });
};

function register() {
      var usn = document.getElementById("register-username").value;
      var psw = document.getElementById("register-password").value;
      var fsn = document.getElementById("register-firstname").value;
      var lsn = document.getElementById("register-lastname").value;
      var eml = document.getElementById("register-email").value;
  var reguser = {username :usn,password:psw ,firstname :fsn,lastname:lsn ,email:eml};
   $.ajax({
        type: "POST",
        data :JSON.stringify(reguser),
        url: "/users/register",
        contentType: "application/json",
        success:function(response){
                document.getElementById("reg_info").innerText=response ;
                },
        error: function(jqXHR, textStatus, errorThrown) {
                alert(JSON.parse(jqXHR.responseText));
                
            }
   });
};

