$(document).ready(function () {
    $('#Header').css('height', $(window).height()*0.6+'px');
    $('#MainContent').css('min-height', $(window).height()+'px');
});

$(window).resize(function () {
    $('#Header').css('height', $(window).height()*0.6+'px');
    $('#MainContent').css('min-height', $(window).height()+'px');
});

