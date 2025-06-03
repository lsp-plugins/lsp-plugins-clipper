<?php
	plugin_header();
	$m      =   ($PAGE == 'clipper_mono') ? 'm' : 's';
?>

<p>
	This plugin allows to drive much more loudness by cutting extra peaks and reach extreme loudness
	levels at the output. The key features are following.
</p>
<p>
	<b>Dithering</b> allows to add some dithering noise to the signal and make the final mix more detailed.
	<b>Loudness limiting</b> option allows to control the input loudness, per-band loudness and input loudness at the
	output clipper.
	<b>Overdrive protection</b> allows to add short-time compression to the signal to make clipping effect less noticeable.
	<b>Multiple sigmoid functions</b> allow to select the best sounding clipping function.
	<b>Input and output loudness measurements</b> allow to control loudness level of the signal in LUFS.
</p>

<p><b>Controls</b>:</p>
<ul>
	<li><b>Dither</b> - allows to enable dithering noise depending on the bitness of the desired output signal.</li>
	<li><b>ODP</b> - enabled overdrive protection compressor.</li>
	<li><b>Clipping</b> - enables clipping function applied to the signal.</li>
	<li><b>Log Scale</b> - switches clipping function graph representation in linear/logarithmic scale.</li>
	<li><b>Function</b> - clipping function</li>
	<li><b>ODP Thresh</b> - the threshold of the overdrive protection compressor.</li>
	<li><b>ODP Knee</b> - the knee of the overdrive protection compressor.</li>
	<li><b>ODP Meter</b> - the amount of gain reduction applied to the signal while compressing it's peaks.</li>
	<li><b>Clip Thresh</b> - the threshold of the clipping function. Signals below the threshold have constant amplification.</li>
	<li><b>Clip Pumping</b> - additional way to pump the loudness of the band by applying extra amplification and keeping peaks not greater than 0 dB.</li>
	<li><b>Clip Meter</b> - the amount of gain reduction applied at the clipping stage.</li>	
	<li><b>Reactivity</b> - sets up the reactivity of the ODP compressor.</li>
	<?php if ($m == 's') { ?>
	<li><b>Stereo Link</b> - allows to control how the left channel of ODP compressor affects the right channel and vice verse.</li>
	<?php } ?>
	<li><b>Time Graph Gain</b> - the overall band gain reduction meter.</li>
	<li><b>Time Graph dB In</b> - the input signal meter.</li>
	<li><b>Time Graph dB Out</b> - the output signal meter.</li>
	<li><b>Time Graph LUFS In</b> - the input signal LUFS meter.</li>
	<li><b>Time Graph LUFS Out</b> - the output signal LUFS meter.</li>
	<li><b>LUFS Limit</b> button - allows to enable loudness limiting at the input of the clipper.</li>
	<li><b>LUFS Limit</b> knob - allows to set maximum allowed loudness at the input of the clipper.</li>
	<li><b>LUFS Limit</b> meter - the amount of gain reduction applied to the input signal while reducing loudness.</li>
	<li><b>Boost</b> - allows to additionally boost the processed signal to compensate threshold level lower than 0 dB.</li>
	<li><b>Boost Threshold</b> - allows to adjust lower than 0 dB clipping threshold.</li>
</ul>

<p><b>Signal</b> section:</p>
<ul>
	<li><b>Input</b> - additional gain applied to the input signal.</li>
	<li><b>Output</b> - additional gain applied to the output signal.</li>
</ul>

