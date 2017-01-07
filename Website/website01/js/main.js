$(document).ready(function () {
    $('#Header').css('height', $(window).height()*0.6+'px');
});

$(window).resize(function () {
    $('#Header').css('height', $(window).height()*0.6+'px');
});