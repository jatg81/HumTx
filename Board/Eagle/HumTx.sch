<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.2.2">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="24" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="no" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="no" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="no" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="no" active="yes"/>
<layer number="99" name="SpiceOrder" color="5" fill="1" visible="no" active="yes"/>
<layer number="200" name="200bmp" color="14" fill="10" visible="no" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="supply2" urn="urn:adsk.eagle:library:372">
<description>&lt;b&gt;Supply Symbols&lt;/b&gt;&lt;p&gt;
GND, VCC, 0V, +5V, -5V, etc.&lt;p&gt;
Please keep in mind, that these devices are necessary for the
automatic wiring of the supply signals.&lt;p&gt;
The pin name defined in the symbol is identical to the net which is to be wired automatically.&lt;p&gt;
In this library the device names are the same as the pin names of the symbols, therefore the correct signal names appear next to the supply symbols in the schematic.&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="GND" urn="urn:adsk.eagle:symbol:26990/1" library_version="2">
<wire x1="-1.27" y1="0" x2="1.27" y2="0" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="0" y2="-1.27" width="0.254" layer="94"/>
<wire x1="0" y1="-1.27" x2="-1.27" y2="0" width="0.254" layer="94"/>
<text x="-1.905" y="-3.175" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
<symbol name="VCC" urn="urn:adsk.eagle:symbol:26997/1" library_version="2">
<circle x="0" y="1.27" radius="1.27" width="0.254" layer="94"/>
<text x="-1.905" y="3.175" size="1.778" layer="96">&gt;VALUE</text>
<pin name="VCC" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="GND" urn="urn:adsk.eagle:component:27037/1" prefix="SUPPLY" library_version="2">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="GND" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="VCC" urn="urn:adsk.eagle:component:27060/1" prefix="SUPPLY" library_version="2">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="VCC" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="linear" urn="urn:adsk.eagle:library:262">
<description>&lt;b&gt;Linear Devices&lt;/b&gt;&lt;p&gt;
Operational amplifiers,  comparators, voltage regulators, ADCs, DACs, etc.&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="TO92" urn="urn:adsk.eagle:footprint:16150/1" library_version="2">
<description>&lt;b&gt;TO-92&lt;/b&gt;</description>
<wire x1="-2.095" y1="-1.651" x2="-0.7869" y2="2.5484" width="0.1524" layer="21" curve="-111.097684"/>
<wire x1="0.7869" y1="2.5484" x2="2.095" y2="-1.651" width="0.1524" layer="21" curve="-111.097684"/>
<wire x1="-2.095" y1="-1.651" x2="2.095" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-2.254" y1="-0.254" x2="-0.286" y2="-0.254" width="0.1524" layer="51"/>
<wire x1="-2.655" y1="-0.254" x2="-2.254" y2="-0.254" width="0.1524" layer="21"/>
<wire x1="-0.286" y1="-0.254" x2="0.286" y2="-0.254" width="0.1524" layer="21"/>
<wire x1="2.254" y1="-0.254" x2="2.655" y2="-0.254" width="0.1524" layer="21"/>
<wire x1="0.286" y1="-0.254" x2="2.254" y2="-0.254" width="0.1524" layer="51"/>
<wire x1="-0.7864" y1="2.5484" x2="0.7864" y2="2.5484" width="0.1524" layer="51" curve="-34.298964"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="0" y="1.905" drill="0.8128" shape="octagon"/>
<pad name="3" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="2.413" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="2.921" y="-1.27" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="TO220H" urn="urn:adsk.eagle:footprint:16151/1" library_version="2">
<description>&lt;b&gt;TO-220&lt;/b&gt;</description>
<wire x1="-5.207" y1="-7.62" x2="5.207" y2="-7.62" width="0.1524" layer="21"/>
<wire x1="5.207" y1="8.255" x2="-5.207" y2="8.255" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-7.62" x2="5.207" y2="4.826" width="0.1524" layer="21"/>
<wire x1="5.207" y1="4.826" x2="4.318" y2="4.826" width="0.1524" layer="21"/>
<wire x1="4.318" y1="4.826" x2="4.318" y2="6.35" width="0.1524" layer="21"/>
<wire x1="4.318" y1="6.35" x2="5.207" y2="6.35" width="0.1524" layer="21"/>
<wire x1="5.207" y1="6.35" x2="5.207" y2="8.255" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="-7.62" x2="-5.207" y2="4.826" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="4.826" x2="-4.318" y2="4.826" width="0.1524" layer="21"/>
<wire x1="-4.318" y1="4.826" x2="-4.318" y2="6.35" width="0.1524" layer="21"/>
<wire x1="-4.318" y1="6.35" x2="-5.207" y2="6.35" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="6.35" x2="-5.207" y2="8.255" width="0.1524" layer="21"/>
<wire x1="-4.572" y1="-6.985" x2="4.572" y2="-6.985" width="0.0508" layer="21"/>
<wire x1="4.572" y1="1.27" x2="4.572" y2="-6.985" width="0.0508" layer="21"/>
<wire x1="4.572" y1="1.27" x2="-4.572" y2="1.27" width="0.0508" layer="21"/>
<wire x1="-4.572" y1="-6.985" x2="-4.572" y2="1.27" width="0.0508" layer="21"/>
<circle x="0" y="4.826" radius="1.8034" width="0.1524" layer="21"/>
<circle x="0" y="4.826" radius="2.54" width="0" layer="43"/>
<circle x="0" y="4.826" radius="2.54" width="0" layer="42"/>
<pad name="1" x="-2.54" y="-10.16" drill="1.1176" shape="long" rot="R90"/>
<pad name="2" x="0" y="-10.16" drill="1.1176" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="-10.16" drill="1.1176" shape="long" rot="R90"/>
<text x="-5.461" y="-10.922" size="1.778" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="7.366" y="-11.049" size="1.778" layer="27" ratio="10" rot="R90">&gt;VALUE</text>
<rectangle x1="2.159" y1="-11.049" x2="2.921" y2="-10.414" layer="21"/>
<rectangle x1="-0.381" y1="-11.049" x2="0.381" y2="-10.414" layer="21"/>
<rectangle x1="-2.921" y1="-11.049" x2="-2.159" y2="-10.414" layer="21"/>
<rectangle x1="-3.175" y1="-10.414" x2="-1.905" y2="-7.62" layer="21"/>
<rectangle x1="-0.635" y1="-10.414" x2="0.635" y2="-7.62" layer="21"/>
<rectangle x1="1.905" y1="-10.414" x2="3.175" y2="-7.62" layer="21"/>
<hole x="0" y="4.826" drill="3.302"/>
</package>
<package name="TO39" urn="urn:adsk.eagle:footprint:16152/1" library_version="2">
<description>&lt;b&gt;Metal Can Package&lt;/b&gt;</description>
<wire x1="-4.0386" y1="-3.5306" x2="-3.5052" y2="-2.9972" width="0.1524" layer="21"/>
<wire x1="-2.9718" y1="-3.5306" x2="-3.5052" y2="-4.064" width="0.1524" layer="21"/>
<wire x1="-3.5052" y1="-4.064" x2="-4.0386" y2="-3.5306" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="4.572" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="3.8608" width="0.0508" layer="21"/>
<pad name="1" x="0" y="-2.54" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="0" y="2.54" drill="0.8128" shape="octagon"/>
<text x="-2.794" y="4.826" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.302" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="TO252" urn="urn:adsk.eagle:footprint:16153/1" library_version="2">
<description>&lt;b&gt;SMALL OUTLINE TRANSISTOR&lt;/b&gt;&lt;p&gt;
TS-003</description>
<wire x1="3.2766" y1="3.8354" x2="3.277" y2="-2.159" width="0.2032" layer="21"/>
<wire x1="3.277" y1="-2.159" x2="-3.277" y2="-2.159" width="0.2032" layer="21"/>
<wire x1="-3.277" y1="-2.159" x2="-3.2766" y2="3.8354" width="0.2032" layer="21"/>
<wire x1="-3.277" y1="3.835" x2="3.2774" y2="3.8346" width="0.2032" layer="51"/>
<wire x1="-2.5654" y1="3.937" x2="-2.5654" y2="4.6482" width="0.2032" layer="51"/>
<wire x1="-2.5654" y1="4.6482" x2="-2.1082" y2="5.1054" width="0.2032" layer="51"/>
<wire x1="-2.1082" y1="5.1054" x2="2.1082" y2="5.1054" width="0.2032" layer="51"/>
<wire x1="2.1082" y1="5.1054" x2="2.5654" y2="4.6482" width="0.2032" layer="51"/>
<wire x1="2.5654" y1="4.6482" x2="2.5654" y2="3.937" width="0.2032" layer="51"/>
<wire x1="2.5654" y1="3.937" x2="-2.5654" y2="3.937" width="0.2032" layer="51"/>
<smd name="3" x="0" y="2.5" dx="5.4" dy="6.2" layer="1"/>
<smd name="1" x="-2.28" y="-4.8" dx="1" dy="1.6" layer="1"/>
<smd name="2" x="2.28" y="-4.8" dx="1" dy="1.6" layer="1"/>
<text x="-3.81" y="-2.54" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="5.08" y="-2.54" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-2.7178" y1="-5.1562" x2="-1.8542" y2="-2.2606" layer="51"/>
<rectangle x1="1.8542" y1="-5.1562" x2="2.7178" y2="-2.2606" layer="51"/>
<rectangle x1="-0.4318" y1="-3.0226" x2="0.4318" y2="-2.2606" layer="21"/>
<polygon width="0.1998" layer="51">
<vertex x="-2.5654" y="3.937"/>
<vertex x="-2.5654" y="4.6482"/>
<vertex x="-2.1082" y="5.1054"/>
<vertex x="2.1082" y="5.1054"/>
<vertex x="2.5654" y="4.6482"/>
<vertex x="2.5654" y="3.937"/>
</polygon>
</package>
<package name="TO220V" urn="urn:adsk.eagle:footprint:16154/1" library_version="2">
<description>&lt;b&gt;TO 200 vertical&lt;/b&gt;</description>
<wire x1="5.08" y1="-1.143" x2="4.953" y2="-4.064" width="0.127" layer="21"/>
<wire x1="4.699" y1="-4.318" x2="4.953" y2="-4.064" width="0.127" layer="21"/>
<wire x1="4.699" y1="-4.318" x2="-4.699" y2="-4.318" width="0.127" layer="21"/>
<wire x1="-4.953" y1="-4.064" x2="-4.699" y2="-4.318" width="0.127" layer="21"/>
<wire x1="-4.953" y1="-4.064" x2="-5.08" y2="-1.143" width="0.127" layer="21"/>
<circle x="-4.4958" y="-3.7084" radius="0.254" width="0.127" layer="21"/>
<pad name="1" x="-2.54" y="-2.54" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="0" y="-2.54" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="-2.54" drill="1.016" shape="long" rot="R90"/>
<text x="-5.08" y="-6.0452" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-7.62" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-3.175" y="-3.175" size="1.27" layer="51" ratio="10">1</text>
<text x="-0.635" y="-3.175" size="1.27" layer="51" ratio="10">2</text>
<text x="1.905" y="-3.175" size="1.27" layer="51" ratio="10">3</text>
<rectangle x1="-5.334" y1="-0.762" x2="5.334" y2="0" layer="21"/>
<rectangle x1="-5.334" y1="-1.27" x2="-3.429" y2="-0.762" layer="21"/>
<rectangle x1="-1.651" y1="-1.27" x2="-0.889" y2="-0.762" layer="21"/>
<rectangle x1="-3.429" y1="-1.27" x2="-1.651" y2="-0.762" layer="51"/>
<rectangle x1="0.889" y1="-1.27" x2="1.651" y2="-0.762" layer="21"/>
<rectangle x1="3.429" y1="-1.27" x2="5.334" y2="-0.762" layer="21"/>
<rectangle x1="-0.889" y1="-1.27" x2="0.889" y2="-0.762" layer="51"/>
<rectangle x1="1.651" y1="-1.27" x2="3.429" y2="-0.762" layer="51"/>
</package>
</packages>
<packages3d>
<package3d name="TO92" urn="urn:adsk.eagle:package:16416/2" type="model" library_version="2">
<description>TO-92</description>
</package3d>
<package3d name="TO220H" urn="urn:adsk.eagle:package:16414/1" type="box" library_version="2">
<description>TO-220</description>
</package3d>
<package3d name="TO39" urn="urn:adsk.eagle:package:16419/1" type="box" library_version="2">
<description>Metal Can Package</description>
</package3d>
<package3d name="TO252" urn="urn:adsk.eagle:package:16415/2" type="model" library_version="2">
<description>SMALL OUTLINE TRANSISTOR
TS-003</description>
</package3d>
<package3d name="TO220V" urn="urn:adsk.eagle:package:16417/1" type="box" library_version="2">
<description>TO 200 vertical</description>
</package3d>
</packages3d>
<symbols>
<symbol name="78XX" urn="urn:adsk.eagle:symbol:16149/1" library_version="2">
<wire x1="-7.62" y1="-5.08" x2="7.62" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="7.62" y1="-5.08" x2="7.62" y2="2.54" width="0.4064" layer="94"/>
<wire x1="7.62" y1="2.54" x2="-7.62" y2="2.54" width="0.4064" layer="94"/>
<wire x1="-7.62" y1="2.54" x2="-7.62" y2="-5.08" width="0.4064" layer="94"/>
<text x="-7.62" y="5.715" size="1.778" layer="95">&gt;NAME</text>
<text x="-7.62" y="3.175" size="1.778" layer="96">&gt;VALUE</text>
<text x="-2.032" y="-4.318" size="1.524" layer="95">GND</text>
<pin name="VI" x="-10.16" y="0" length="short" direction="in"/>
<pin name="GND" x="0" y="-7.62" visible="pad" length="short" direction="pas" rot="R90"/>
<pin name="VO" x="10.16" y="0" length="short" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="78*" urn="urn:adsk.eagle:component:16686/3" prefix="IC" library_version="2">
<description>Positive &lt;b&gt;VOLTAGE REGULATOR&lt;/b&gt;&lt;p&gt;
Source:&lt;br&gt;
http://cache.national.com/ds/LM/LM78L05.pdf&lt;br&gt;
http://www.fairchildsemi.com/ds/LM/LM7805.pdf</description>
<gates>
<gate name="A1" symbol="78XX" x="0" y="0"/>
</gates>
<devices>
<device name="Z" package="TO92">
<connects>
<connect gate="A1" pin="GND" pad="2"/>
<connect gate="A1" pin="VI" pad="3"/>
<connect gate="A1" pin="VO" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16416/2"/>
</package3dinstances>
<technologies>
<technology name="L05"/>
<technology name="L08"/>
<technology name="L12"/>
<technology name="L15"/>
<technology name="L18"/>
<technology name="L24"/>
</technologies>
</device>
<device name="T" package="TO220H">
<connects>
<connect gate="A1" pin="GND" pad="2"/>
<connect gate="A1" pin="VI" pad="1"/>
<connect gate="A1" pin="VO" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16414/1"/>
</package3dinstances>
<technologies>
<technology name="05"/>
<technology name="06"/>
<technology name="08"/>
<technology name="12"/>
<technology name="15"/>
<technology name="18"/>
<technology name="24"/>
</technologies>
</device>
<device name="H" package="TO39">
<connects>
<connect gate="A1" pin="GND" pad="3"/>
<connect gate="A1" pin="VI" pad="1"/>
<connect gate="A1" pin="VO" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16419/1"/>
</package3dinstances>
<technologies>
<technology name="05"/>
<technology name="06"/>
<technology name="08"/>
<technology name="12"/>
<technology name="15"/>
<technology name="18"/>
<technology name="24"/>
</technologies>
</device>
<device name="L" package="TO92">
<connects>
<connect gate="A1" pin="GND" pad="2"/>
<connect gate="A1" pin="VI" pad="3"/>
<connect gate="A1" pin="VO" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16416/2"/>
</package3dinstances>
<technologies>
<technology name="05"/>
<technology name="06"/>
<technology name="08"/>
<technology name="12"/>
<technology name="15"/>
<technology name="18"/>
<technology name="24"/>
</technologies>
</device>
<device name="DT" package="TO252">
<connects>
<connect gate="A1" pin="GND" pad="3"/>
<connect gate="A1" pin="VI" pad="1"/>
<connect gate="A1" pin="VO" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16415/2"/>
</package3dinstances>
<technologies>
<technology name="05"/>
<technology name="06"/>
<technology name="08"/>
<technology name="12"/>
<technology name="15"/>
<technology name="18"/>
<technology name="24"/>
</technologies>
</device>
<device name="TV" package="TO220V">
<connects>
<connect gate="A1" pin="GND" pad="2"/>
<connect gate="A1" pin="VI" pad="1"/>
<connect gate="A1" pin="VO" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:16417/1"/>
</package3dinstances>
<technologies>
<technology name="05"/>
<technology name="12"/>
<technology name="15"/>
<technology name="18"/>
<technology name="24"/>
<technology name="6"/>
<technology name="8"/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="jst">
<packages>
<package name="B6B-ZR">
<wire x1="0" y1="0.07" x2="10.5" y2="0.07" width="0.127" layer="21"/>
<wire x1="10.5" y1="0.07" x2="10.5" y2="-3.43" width="0.127" layer="21"/>
<wire x1="10.5" y1="-3.43" x2="0" y2="-3.43" width="0.127" layer="21"/>
<wire x1="0" y1="-3.43" x2="0" y2="0.07" width="0.127" layer="21"/>
<text x="-0.75" y="0.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.75" y="-5.365" size="1.27" layer="27">&gt;VALUE</text>
<pad name="6" x="1.5" y="-1.3" drill="0.5" diameter="1" shape="long" rot="R90"/>
<pad name="5" x="3" y="-1.3" drill="0.5" diameter="1" shape="long" rot="R90"/>
<pad name="4" x="4.5" y="-1.3" drill="0.5" diameter="1" shape="long" rot="R90"/>
<pad name="3" x="6" y="-1.3" drill="0.5" diameter="1" shape="long" rot="R90"/>
<pad name="2" x="7.5" y="-1.3" drill="0.5" diameter="1" shape="long" rot="R90"/>
<pad name="1" x="9" y="-1.3" drill="0.5" diameter="1" shape="long" rot="R90"/>
</package>
</packages>
<symbols>
<symbol name="B6B-ZR">
<pin name="1" x="-5.08" y="0" visible="pin" length="middle"/>
<pin name="2" x="-5.08" y="-2.54" visible="pin" length="middle"/>
<pin name="3" x="-5.08" y="-5.08" visible="pin" length="middle"/>
<pin name="4" x="-5.08" y="-7.62" visible="pin" length="middle"/>
<pin name="5" x="-5.08" y="-10.16" visible="pin" length="middle"/>
<pin name="6" x="-5.08" y="-12.7" visible="pin" length="middle"/>
<text x="0" y="3.302" size="1.27" layer="95">&gt;NAME</text>
<text x="0" y="-17.018" size="1.27" layer="96">&gt;VALUE</text>
<wire x1="-2.54" y1="2.54" x2="5.08" y2="2.54" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-15.24" width="0.254" layer="94"/>
<wire x1="5.08" y1="-15.24" x2="-2.54" y2="-15.24" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-15.24" x2="-2.54" y2="2.54" width="0.254" layer="94"/>
<circle x="0" y="0" radius="0.915809375" width="0.254" layer="94"/>
<circle x="0" y="-2.54" radius="0.915809375" width="0.254" layer="94"/>
<circle x="0" y="-5.08" radius="0.915809375" width="0.254" layer="94"/>
<circle x="0" y="-7.62" radius="0.915809375" width="0.254" layer="94"/>
<circle x="0" y="-10.16" radius="0.915809375" width="0.254" layer="94"/>
<circle x="0" y="-12.7" radius="0.915809375" width="0.254" layer="94"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="B6B-ZR" prefix="CON">
<description>&lt;b&gt;JST-ZH Connector&lt;/b&gt;&lt;br&gt;
6 pol, 1.5mm pitch&lt;br&gt;
Package: &lt;b&gt;B6B-ZR&lt;/b&gt;
&lt;br&gt;&lt;br&gt;
Source: &lt;a href="http://www.jst-mfg.com/product/pdf/eng/eZH.pdf"&gt;http://www.jst-mfg.com/product/pdf/eng/eZH.pdf&lt;/a&gt;</description>
<gates>
<gate name="G$1" symbol="B6B-ZR" x="-2.54" y="5.08"/>
</gates>
<devices>
<device name="" package="B6B-ZR">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="jstmod">
<packages>
<package name="B5B-ZR">
<wire x1="0" y1="0.07" x2="9" y2="0.07" width="0.127" layer="21"/>
<wire x1="9" y1="0.07" x2="9" y2="-3.43" width="0.127" layer="21"/>
<wire x1="9" y1="-3.43" x2="0" y2="-3.43" width="0.127" layer="21"/>
<wire x1="0" y1="-3.43" x2="0" y2="0.07" width="0.127" layer="21"/>
<text x="-0.75" y="0.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.75" y="-5.365" size="1.27" layer="27">&gt;VALUE</text>
<pad name="5" x="1.5" y="-1.3" drill="0.5" diameter="1.1" shape="long" rot="R90"/>
<pad name="4" x="3" y="-1.3" drill="0.5" diameter="1.1" shape="long" rot="R90"/>
<pad name="3" x="4.5" y="-1.3" drill="0.5" diameter="1.1" shape="long" rot="R90"/>
<pad name="2" x="6" y="-1.3" drill="0.5" diameter="1.1" shape="long" rot="R90"/>
<pad name="1" x="7.5" y="-1.3" drill="0.5" diameter="1.1" shape="long" rot="R90"/>
</package>
<package name="B3B-ZR">
<description>&lt;b&gt;JST ZH Connector&lt;/b&gt;
Top Entry type&lt;br&gt;
3 pol, 1.5 mm Pitch&lt;br&gt;
Source: &lt;a href="http://www.jst-mfg.com/product/pdf/eng/eZH.pdf"&gt;http://www.jst-mfg.com/product/pdf/eng/eZH.pdf&lt;/a&gt;</description>
<wire x1="-3" y1="1.3" x2="3" y2="1.3" width="0.127" layer="21"/>
<wire x1="3" y1="1.3" x2="3" y2="-2.2" width="0.127" layer="21"/>
<wire x1="3" y1="-2.2" x2="-3" y2="-2.2" width="0.127" layer="21"/>
<wire x1="-3" y1="-2.2" x2="-3" y2="1.3" width="0.127" layer="21"/>
<pad name="1" x="-1.5" y="0" drill="0.5" diameter="1.1" shape="long" rot="R90"/>
<pad name="2" x="0" y="0" drill="0.5" diameter="1.1" shape="long" rot="R90"/>
<pad name="3" x="1.5" y="0" drill="0.5" diameter="1.1" shape="long" rot="R90"/>
<text x="-3.75" y="1.5" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.75" y="-3.5" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="B4B-ZR">
<description>&lt;b&gt;ZH CONNECTOR&lt;/b&gt;  Top entry type, 1.5 mm, 4 pin 1 row&lt;p&gt;
Source: http://www.jst.com .. eZH.pdf</description>
<wire x1="0" y1="0.07" x2="7.5" y2="0.07" width="0.127" layer="21"/>
<wire x1="7.5" y1="0.07" x2="7.5" y2="-3.43" width="0.127" layer="21"/>
<wire x1="7.5" y1="-3.43" x2="0" y2="-3.43" width="0.127" layer="21"/>
<wire x1="0" y1="-3.43" x2="0" y2="0.07" width="0.127" layer="21"/>
<text x="-0.75" y="0.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.75" y="-5.365" size="1.27" layer="27">&gt;VALUE</text>
<pad name="4" x="1.5" y="-1.3" drill="0.5" diameter="1.1" shape="long" rot="R90"/>
<pad name="3" x="3" y="-1.3" drill="0.5" diameter="1.1" shape="long" rot="R90"/>
<pad name="2" x="4.5" y="-1.3" drill="0.5" diameter="1.1" shape="long" rot="R90"/>
<pad name="1" x="6" y="-1.3" drill="0.5" diameter="1.1" shape="long" rot="R90"/>
</package>
</packages>
<symbols>
<symbol name="B5B-ZR">
<pin name="1" x="-7.62" y="5.08" visible="pin" length="middle"/>
<pin name="2" x="-7.62" y="2.54" visible="pin" length="middle"/>
<pin name="3" x="-7.62" y="0" visible="pin" length="middle"/>
<pin name="4" x="-7.62" y="-2.54" visible="pin" length="middle"/>
<pin name="5" x="-7.62" y="-5.08" visible="pin" length="middle"/>
<text x="-2.54" y="8.382" size="1.27" layer="95">&gt;NAME</text>
<text x="-2.54" y="-9.398" size="1.27" layer="96">&gt;VALUE</text>
<wire x1="-5.08" y1="7.62" x2="2.54" y2="7.62" width="0.254" layer="94"/>
<wire x1="2.54" y1="7.62" x2="2.54" y2="-7.62" width="0.254" layer="94"/>
<wire x1="2.54" y1="-7.62" x2="-5.08" y2="-7.62" width="0.254" layer="94"/>
<wire x1="-5.08" y1="-7.62" x2="-5.08" y2="7.62" width="0.254" layer="94"/>
<circle x="-2.54" y="5.08" radius="0.915809375" width="0.254" layer="94"/>
<circle x="-2.54" y="2.54" radius="0.915809375" width="0.254" layer="94"/>
<circle x="-2.54" y="0" radius="0.915809375" width="0.254" layer="94"/>
<circle x="-2.54" y="-2.54" radius="0.915809375" width="0.254" layer="94"/>
<circle x="-2.54" y="-5.08" radius="0.915809375" width="0.254" layer="94"/>
</symbol>
<symbol name="B3B-ZR">
<pin name="1" x="-7.62" y="2.54" visible="pin" length="middle"/>
<pin name="2" x="-7.62" y="0" visible="pin" length="middle"/>
<pin name="3" x="-7.62" y="-2.54" visible="pin" length="middle"/>
<text x="-2.54" y="5.842" size="1.27" layer="95">&gt;NAME</text>
<text x="-2.54" y="-6.858" size="1.27" layer="96">&gt;VALUE</text>
<wire x1="-5.08" y1="5.08" x2="2.54" y2="5.08" width="0.254" layer="94"/>
<wire x1="2.54" y1="5.08" x2="2.54" y2="-5.08" width="0.254" layer="94"/>
<wire x1="2.54" y1="-5.08" x2="-5.08" y2="-5.08" width="0.254" layer="94"/>
<wire x1="-5.08" y1="-5.08" x2="-5.08" y2="5.08" width="0.254" layer="94"/>
<circle x="-2.54" y="2.54" radius="0.915809375" width="0.254" layer="94"/>
<circle x="-2.54" y="0" radius="0.915809375" width="0.254" layer="94"/>
<circle x="-2.54" y="-2.54" radius="0.915809375" width="0.254" layer="94"/>
</symbol>
<symbol name="B4B-ZR">
<pin name="1" x="-7.62" y="2.54" visible="pin" length="middle"/>
<pin name="2" x="-7.62" y="0" visible="pin" length="middle"/>
<pin name="3" x="-7.62" y="-2.54" visible="pin" length="middle"/>
<pin name="4" x="-7.62" y="-5.08" visible="pin" length="middle"/>
<text x="-2.54" y="5.842" size="1.27" layer="95">&gt;NAME</text>
<text x="-2.54" y="-9.398" size="1.27" layer="96">&gt;VALUE</text>
<wire x1="-5.08" y1="5.08" x2="2.54" y2="5.08" width="0.254" layer="94"/>
<wire x1="2.54" y1="5.08" x2="2.54" y2="-7.62" width="0.254" layer="94"/>
<wire x1="2.54" y1="-7.62" x2="-5.08" y2="-7.62" width="0.254" layer="94"/>
<wire x1="-5.08" y1="-7.62" x2="-5.08" y2="5.08" width="0.254" layer="94"/>
<circle x="-2.54" y="2.54" radius="0.915809375" width="0.254" layer="94"/>
<circle x="-2.54" y="0" radius="0.915809375" width="0.254" layer="94"/>
<circle x="-2.54" y="-2.54" radius="0.915809375" width="0.254" layer="94"/>
<circle x="-2.54" y="-5.08" radius="0.915809375" width="0.254" layer="94"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="B5B-ZR">
<description>&lt;b&gt;JST-ZH Connector&lt;/b&gt;&lt;br&gt;
5 pol, 1.5mm pitch&lt;br&gt;
Package: &lt;b&gt;B5B-ZR&lt;/b&gt;
&lt;br&gt;&lt;br&gt;
Source: &lt;a href="http://www.jst-mfg.com/product/pdf/eng/eZH.pdf"&gt;http://www.jst-mfg.com/product/pdf/eng/eZH.pdf&lt;/a&gt;</description>
<gates>
<gate name="G$1" symbol="B5B-ZR" x="0" y="0"/>
</gates>
<devices>
<device name="" package="B5B-ZR">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="B3B-ZR" prefix="CON">
<description>&lt;b&gt;JST-ZH Connector&lt;/b&gt;&lt;br&gt;
3 pol, 1.5mm pitch&lt;br&gt;
Package: &lt;b&gt;B3B-ZR&lt;/b&gt;
&lt;br&gt;&lt;br&gt;
Source: &lt;a href="http://www.jst-mfg.com/product/pdf/eng/eZH.pdf"&gt;http://www.jst-mfg.com/product/pdf/eng/eZH.pdf&lt;/a&gt;</description>
<gates>
<gate name="G$1" symbol="B3B-ZR" x="0" y="0"/>
</gates>
<devices>
<device name="" package="B3B-ZR">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="B4B-ZR" prefix="CON">
<description>&lt;b&gt;JST-ZH Connector&lt;/b&gt;&lt;br&gt;
4 pol, 1.5mm pitch&lt;br&gt;
Package: &lt;b&gt;B4B-ZR&lt;/b&gt;
&lt;br&gt;&lt;br&gt;
Source: &lt;a href="http://www.jst-mfg.com/product/pdf/eng/eZH.pdf"&gt;http://www.jst-mfg.com/product/pdf/eng/eZH.pdf&lt;/a&gt;</description>
<gates>
<gate name="G$1" symbol="B4B-ZR" x="0" y="0"/>
</gates>
<devices>
<device name="" package="B4B-ZR">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="transistor-power" urn="urn:adsk.eagle:library:400">
<description>&lt;b&gt;Power Transistors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="D2PACK" urn="urn:adsk.eagle:footprint:29392/1" library_version="2">
<description>&lt;b&gt;D2PACK&lt;/b&gt;&lt;p&gt;
INTERNATIONAL RECTIFIER, irg4bc15ud-s.pdf</description>
<wire x1="-5.1308" y1="-4.0894" x2="5.1308" y2="-4.0894" width="0.254" layer="21"/>
<wire x1="5.1308" y1="-4.0894" x2="5.1308" y2="4.445" width="0.254" layer="51"/>
<wire x1="5.1308" y1="4.445" x2="3.1242" y2="5.8166" width="0.254" layer="51"/>
<wire x1="3.1242" y1="5.8166" x2="-3.3782" y2="5.8166" width="0.254" layer="51"/>
<wire x1="-3.3782" y1="5.8166" x2="-5.1308" y2="4.699" width="0.254" layer="51"/>
<wire x1="-5.1308" y1="4.699" x2="-5.1308" y2="4.445" width="0.254" layer="51"/>
<wire x1="-5.1308" y1="4.445" x2="-5.1308" y2="-4.0894" width="0.254" layer="51"/>
<wire x1="-5.1308" y1="4.445" x2="5.1308" y2="4.445" width="0.254" layer="51"/>
<wire x1="-5.1308" y1="-4.0894" x2="-5.1308" y2="-2.3114" width="0.254" layer="21"/>
<wire x1="5.1308" y1="-4.0894" x2="5.1308" y2="-2.3114" width="0.254" layer="21"/>
<smd name="1" x="-2.54" y="-8.89" dx="2.0828" dy="3.81" layer="1"/>
<smd name="3" x="2.54" y="-8.89" dx="2.0828" dy="3.81" layer="1"/>
<smd name="2" x="0" y="2.54" dx="11.43" dy="8.89" layer="1"/>
<text x="-5.588" y="7.239" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.81" y="-3.429" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.0988" y1="-9.525" x2="-1.9812" y2="-8.1026" layer="51"/>
<rectangle x1="-3.2512" y1="-8.1534" x2="-1.8288" y2="-6.731" layer="51"/>
<rectangle x1="-3.2512" y1="-6.731" x2="-1.8288" y2="-4.2418" layer="21"/>
<rectangle x1="1.9812" y1="-9.525" x2="3.0988" y2="-8.1026" layer="51"/>
<rectangle x1="1.8288" y1="-8.1534" x2="3.2512" y2="-6.731" layer="51"/>
<rectangle x1="1.8288" y1="-6.731" x2="3.2512" y2="-4.2418" layer="21"/>
<rectangle x1="-0.7112" y1="-6.731" x2="0.7112" y2="-4.2418" layer="21"/>
</package>
<package name="TO220AH" urn="urn:adsk.eagle:footprint:29379/1" library_version="2">
<description>&lt;b&gt;Molded Package&lt;/b&gt;&lt;p&gt;
grid 2.54 mm</description>
<wire x1="-5.207" y1="-1.27" x2="5.207" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.207" y1="14.605" x2="-5.207" y2="14.605" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-1.27" x2="5.207" y2="11.176" width="0.1524" layer="21"/>
<wire x1="5.207" y1="11.176" x2="4.318" y2="11.176" width="0.1524" layer="21"/>
<wire x1="4.318" y1="11.176" x2="4.318" y2="12.7" width="0.1524" layer="21"/>
<wire x1="4.318" y1="12.7" x2="5.207" y2="12.7" width="0.1524" layer="21"/>
<wire x1="5.207" y1="12.7" x2="5.207" y2="14.605" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="-1.27" x2="-5.207" y2="11.176" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="11.176" x2="-4.318" y2="11.176" width="0.1524" layer="21"/>
<wire x1="-4.318" y1="11.176" x2="-4.318" y2="12.7" width="0.1524" layer="21"/>
<wire x1="-4.318" y1="12.7" x2="-5.207" y2="12.7" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="12.7" x2="-5.207" y2="14.605" width="0.1524" layer="21"/>
<wire x1="-4.572" y1="-0.635" x2="4.572" y2="-0.635" width="0.0508" layer="21"/>
<wire x1="4.572" y1="7.62" x2="4.572" y2="-0.635" width="0.0508" layer="21"/>
<wire x1="4.572" y1="7.62" x2="-4.572" y2="7.62" width="0.0508" layer="21"/>
<wire x1="-4.572" y1="-0.635" x2="-4.572" y2="7.62" width="0.0508" layer="21"/>
<circle x="0" y="11.176" radius="1.8034" width="0.1524" layer="21"/>
<circle x="0" y="11.176" radius="4.191" width="0" layer="42"/>
<circle x="0" y="11.176" radius="4.191" width="0" layer="43"/>
<pad name="B" x="-2.54" y="-6.35" drill="1.1176" shape="long" rot="R90"/>
<pad name="C" x="0" y="-6.35" drill="1.1176" shape="long" rot="R90"/>
<pad name="E" x="2.54" y="-6.35" drill="1.1176" shape="long" rot="R90"/>
<text x="-3.81" y="5.207" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.937" y="2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
<text x="-4.445" y="7.874" size="1.016" layer="21" ratio="10">A17,5mm</text>
<rectangle x1="2.159" y1="-4.699" x2="2.921" y2="-4.064" layer="21"/>
<rectangle x1="-0.381" y1="-4.699" x2="0.381" y2="-4.064" layer="21"/>
<rectangle x1="-2.921" y1="-4.699" x2="-2.159" y2="-4.064" layer="21"/>
<rectangle x1="-3.175" y1="-4.064" x2="-1.905" y2="-1.27" layer="21"/>
<rectangle x1="-0.635" y1="-4.064" x2="0.635" y2="-1.27" layer="21"/>
<rectangle x1="1.905" y1="-4.064" x2="3.175" y2="-1.27" layer="21"/>
<rectangle x1="-2.921" y1="-6.604" x2="-2.159" y2="-4.699" layer="51"/>
<rectangle x1="-0.381" y1="-6.604" x2="0.381" y2="-4.699" layer="51"/>
<rectangle x1="2.159" y1="-6.604" x2="2.921" y2="-4.699" layer="51"/>
<hole x="0" y="11.176" drill="3.302"/>
</package>
<package name="TO262-H" urn="urn:adsk.eagle:footprint:29394/1" library_version="2">
<description>&lt;b&gt;Molded Package&lt;/b&gt;</description>
<wire x1="-5.1308" y1="-4.9784" x2="5.1308" y2="-4.9784" width="0.254" layer="21"/>
<wire x1="5.1308" y1="-4.9784" x2="5.1308" y2="3.937" width="0.254" layer="21"/>
<wire x1="5.1308" y1="3.937" x2="3.1242" y2="5.3086" width="0.254" layer="21"/>
<wire x1="3.1242" y1="5.3086" x2="-3.3782" y2="5.3086" width="0.254" layer="21"/>
<wire x1="-3.3782" y1="5.3086" x2="-5.1308" y2="4.191" width="0.254" layer="21"/>
<wire x1="-5.1308" y1="4.191" x2="-5.1308" y2="3.937" width="0.254" layer="21"/>
<wire x1="-5.1308" y1="3.937" x2="-5.1308" y2="-4.9784" width="0.254" layer="21"/>
<wire x1="-5.1308" y1="3.937" x2="5.1308" y2="3.937" width="0.254" layer="21"/>
<pad name="1" x="-2.54" y="-12.7" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="0" y="-12.7" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="-12.7" drill="1.016" shape="long" rot="R90"/>
<text x="-5.588" y="6.35" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.81" y="-4.318" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-3.2512" y1="-9.0424" x2="-1.8288" y2="-7.62" layer="21"/>
<rectangle x1="-3.2512" y1="-7.62" x2="-1.8288" y2="-5.1308" layer="21"/>
<rectangle x1="1.8288" y1="-7.62" x2="3.2512" y2="-5.1308" layer="21"/>
<rectangle x1="-0.7112" y1="-7.62" x2="0.7112" y2="-5.1308" layer="21"/>
<rectangle x1="-0.7112" y1="-9.0424" x2="0.7112" y2="-7.62" layer="21"/>
<rectangle x1="1.8288" y1="-9.0424" x2="3.2512" y2="-7.62" layer="21"/>
<rectangle x1="-2.921" y1="-12.954" x2="-2.159" y2="-9.017" layer="51"/>
<rectangle x1="-0.381" y1="-12.954" x2="0.381" y2="-9.017" layer="51"/>
<rectangle x1="2.159" y1="-12.954" x2="2.921" y2="-9.017" layer="51"/>
</package>
<package name="TO262-V" urn="urn:adsk.eagle:footprint:29393/1" library_version="2">
<description>&lt;b&gt;Molded Package&lt;/b&gt;</description>
<wire x1="-4.6228" y1="-1.3716" x2="4.6228" y2="-1.3716" width="0.254" layer="21"/>
<wire x1="4.6228" y1="-1.3716" x2="5.1308" y2="0.1524" width="0.254" layer="21"/>
<wire x1="5.1308" y1="0.1524" x2="5.1308" y2="1.7526" width="0.254" layer="21"/>
<wire x1="-5.1308" y1="1.7526" x2="-5.1308" y2="0.1524" width="0.254" layer="21"/>
<wire x1="-5.1308" y1="0.1524" x2="-4.6228" y2="-1.3716" width="0.254" layer="21"/>
<wire x1="-5.1308" y1="1.7526" x2="5.1308" y2="1.7526" width="0.254" layer="21"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="0" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-4.953" y="-3.175" size="1.27" layer="25">&gt;NAME</text>
<text x="-5.08" y="3.937" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-5.2832" y1="1.778" x2="5.2832" y2="3.2004" layer="21"/>
</package>
<package name="TO220BH" urn="urn:adsk.eagle:footprint:29377/1" library_version="2">
<description>&lt;b&gt;Molded Package&lt;/b&gt;&lt;p&gt;
grid 2.54 mm</description>
<wire x1="-5.207" y1="-1.27" x2="5.207" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.207" y1="14.605" x2="-5.207" y2="14.605" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-1.27" x2="5.207" y2="11.176" width="0.1524" layer="21"/>
<wire x1="5.207" y1="11.176" x2="4.318" y2="11.176" width="0.1524" layer="21"/>
<wire x1="4.318" y1="11.176" x2="4.318" y2="12.7" width="0.1524" layer="21"/>
<wire x1="4.318" y1="12.7" x2="5.207" y2="12.7" width="0.1524" layer="21"/>
<wire x1="5.207" y1="12.7" x2="5.207" y2="14.605" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="-1.27" x2="-5.207" y2="11.176" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="11.176" x2="-4.318" y2="11.176" width="0.1524" layer="21"/>
<wire x1="-4.318" y1="11.176" x2="-4.318" y2="12.7" width="0.1524" layer="21"/>
<wire x1="-4.318" y1="12.7" x2="-5.207" y2="12.7" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="12.7" x2="-5.207" y2="14.605" width="0.1524" layer="21"/>
<wire x1="-4.572" y1="-0.635" x2="4.572" y2="-0.635" width="0.0508" layer="21"/>
<wire x1="4.572" y1="7.62" x2="4.572" y2="-0.635" width="0.0508" layer="21"/>
<wire x1="4.572" y1="7.62" x2="-4.572" y2="7.62" width="0.0508" layer="21"/>
<wire x1="-4.572" y1="-0.635" x2="-4.572" y2="7.62" width="0.0508" layer="21"/>
<circle x="0" y="11.176" radius="1.8034" width="0.1524" layer="21"/>
<circle x="0" y="11.176" radius="4.191" width="0" layer="42"/>
<circle x="0" y="11.176" radius="4.191" width="0" layer="43"/>
<pad name="G" x="-2.54" y="-6.35" drill="1.1176" shape="long" rot="R90"/>
<pad name="D" x="0" y="-6.35" drill="1.1176" shape="long" rot="R90"/>
<pad name="S" x="2.54" y="-6.35" drill="1.1176" shape="long" rot="R90"/>
<text x="-3.81" y="5.207" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.937" y="2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
<text x="-4.445" y="7.874" size="1.016" layer="21" ratio="10">A17,5mm</text>
<rectangle x1="2.159" y1="-4.699" x2="2.921" y2="-4.064" layer="21"/>
<rectangle x1="-0.381" y1="-4.699" x2="0.381" y2="-4.064" layer="21"/>
<rectangle x1="-2.921" y1="-4.699" x2="-2.159" y2="-4.064" layer="21"/>
<rectangle x1="-3.175" y1="-4.064" x2="-1.905" y2="-1.27" layer="21"/>
<rectangle x1="-0.635" y1="-4.064" x2="0.635" y2="-1.27" layer="21"/>
<rectangle x1="1.905" y1="-4.064" x2="3.175" y2="-1.27" layer="21"/>
<rectangle x1="-2.921" y1="-6.604" x2="-2.159" y2="-4.699" layer="51"/>
<rectangle x1="-0.381" y1="-6.604" x2="0.381" y2="-4.699" layer="51"/>
<rectangle x1="2.159" y1="-6.604" x2="2.921" y2="-4.699" layer="51"/>
<hole x="0" y="11.176" drill="3.302"/>
</package>
</packages>
<packages3d>
<package3d name="D2PACK" urn="urn:adsk.eagle:package:29497/2" type="model" library_version="2">
<description>D2PACK
INTERNATIONAL RECTIFIER, irg4bc15ud-s.pdf</description>
<packageinstances>
<packageinstance name="D2PACK"/>
</packageinstances>
</package3d>
<package3d name="TO220AH" urn="urn:adsk.eagle:package:29488/1" type="box" library_version="2">
<description>Molded Package
grid 2.54 mm</description>
<packageinstances>
<packageinstance name="TO220AH"/>
</packageinstances>
</package3d>
<package3d name="TO262-H" urn="urn:adsk.eagle:package:29499/1" type="box" library_version="2">
<description>Molded Package</description>
<packageinstances>
<packageinstance name="TO262-H"/>
</packageinstances>
</package3d>
<package3d name="TO262-V" urn="urn:adsk.eagle:package:29496/2" type="model" library_version="2">
<description>Molded Package</description>
<packageinstances>
<packageinstance name="TO262-V"/>
</packageinstances>
</package3d>
<package3d name="TO220BH" urn="urn:adsk.eagle:package:29490/1" type="box" library_version="2">
<description>Molded Package
grid 2.54 mm</description>
<packageinstances>
<packageinstance name="TO220BH"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="HEXFET_N" urn="urn:adsk.eagle:symbol:29402/1" library_version="2">
<wire x1="-1.1176" y1="2.413" x2="-1.1176" y2="-2.54" width="0.254" layer="94"/>
<wire x1="-1.1176" y1="-2.54" x2="-2.54" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="2.54" y1="2.54" x2="2.54" y2="1.905" width="0.1524" layer="94"/>
<wire x1="2.54" y1="0" x2="2.54" y2="-1.905" width="0.1524" layer="94"/>
<wire x1="0.508" y1="-1.905" x2="2.54" y2="-1.905" width="0.1524" layer="94"/>
<wire x1="2.54" y1="-1.905" x2="2.54" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="1.651" y1="0" x2="2.54" y2="0" width="0.1524" layer="94"/>
<wire x1="3.302" y1="0.508" x2="3.81" y2="0.508" width="0.1524" layer="94"/>
<wire x1="3.81" y1="0.508" x2="4.318" y2="0.508" width="0.1524" layer="94"/>
<wire x1="3.81" y1="1.905" x2="2.54" y2="1.905" width="0.1524" layer="94"/>
<wire x1="2.54" y1="1.905" x2="0.5334" y2="1.905" width="0.1524" layer="94"/>
<wire x1="3.81" y1="0.508" x2="3.81" y2="1.905" width="0.1524" layer="94"/>
<wire x1="3.81" y1="-1.905" x2="3.81" y2="-0.127" width="0.1524" layer="94"/>
<wire x1="3.81" y1="-1.905" x2="2.54" y2="-1.905" width="0.1524" layer="94"/>
<circle x="2.54" y="-1.905" radius="0.127" width="0.4064" layer="94"/>
<text x="6.35" y="2.54" size="1.778" layer="95">&gt;NAME</text>
<text x="6.35" y="0" size="1.778" layer="96">&gt;VALUE</text>
<rectangle x1="-0.254" y1="-2.54" x2="0.508" y2="-1.27" layer="94"/>
<rectangle x1="-0.254" y1="1.27" x2="0.508" y2="2.54" layer="94"/>
<rectangle x1="-0.254" y1="-0.889" x2="0.508" y2="0.889" layer="94"/>
<pin name="G" x="-2.54" y="-2.54" visible="off" length="point" direction="pas"/>
<pin name="D" x="2.54" y="5.08" visible="off" length="short" direction="pas" rot="R270"/>
<pin name="S" x="2.54" y="-5.08" visible="off" length="short" direction="pas" rot="R90"/>
<polygon width="0.1524" layer="94">
<vertex x="0.508" y="0"/>
<vertex x="1.778" y="-0.508"/>
<vertex x="1.778" y="0.508"/>
</polygon>
<polygon width="0.1524" layer="94">
<vertex x="3.81" y="0.508"/>
<vertex x="3.302" y="-0.254"/>
<vertex x="4.318" y="-0.254"/>
</polygon>
</symbol>
</symbols>
<devicesets>
<deviceset name="IRF3704" urn="urn:adsk.eagle:component:29580/3" prefix="Q" library_version="2">
<description>&lt;b&gt;SMPS MOSFET&lt;/b&gt;&lt;p&gt;
HEXFET® Power MOSFET&lt;br&gt;
Source: http://www.irf.com/product-info/datasheets/data/irf3704.pdf</description>
<gates>
<gate name="G$1" symbol="HEXFET_N" x="0" y="0"/>
</gates>
<devices>
<device name="S" package="D2PACK">
<connects>
<connect gate="G$1" pin="D" pad="2"/>
<connect gate="G$1" pin="G" pad="1"/>
<connect gate="G$1" pin="S" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29497/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="" package="TO220AH">
<connects>
<connect gate="G$1" pin="D" pad="C"/>
<connect gate="G$1" pin="G" pad="B"/>
<connect gate="G$1" pin="S" pad="E"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29488/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="L" package="TO262-H">
<connects>
<connect gate="G$1" pin="D" pad="2"/>
<connect gate="G$1" pin="G" pad="1"/>
<connect gate="G$1" pin="S" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29499/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="L-V" package="TO262-V">
<connects>
<connect gate="G$1" pin="D" pad="2"/>
<connect gate="G$1" pin="G" pad="1"/>
<connect gate="G$1" pin="S" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29496/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="-H" package="TO220BH">
<connects>
<connect gate="G$1" pin="D" pad="D"/>
<connect gate="G$1" pin="G" pad="G"/>
<connect gate="G$1" pin="S" pad="S"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29490/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="ledmod">
<description>&lt;b&gt;LEDs&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;&lt;br&gt;
Extended by Federico Battaglin &lt;author&gt;&amp;lt;federico.rd@fdpinternational.com&amp;gt;&lt;/author&gt; with DUOLED</description>
<packages>
<package name="LED5MM" urn="urn:adsk.eagle:footprint:15655/1" locally_modified="yes">
<description>&lt;B&gt;LED&lt;/B&gt;&lt;p&gt;
5 mm, round</description>
<wire x1="2.54" y1="-1.905" x2="2.54" y2="1.905" width="0.2032" layer="21"/>
<wire x1="2.54" y1="-1.905" x2="2.54" y2="1.905" width="0.254" layer="21" curve="-286.260205"/>
<wire x1="-1.143" y1="0" x2="0" y2="1.143" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-1.143" x2="1.143" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="-1.651" y1="0" x2="0" y2="1.651" width="0.254" layer="51" curve="-90"/>
<wire x1="0" y1="-1.651" x2="1.651" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="-2.159" y1="0" x2="0" y2="2.159" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-2.159" x2="2.159" y2="0" width="0.1524" layer="51" curve="90"/>
<circle x="0" y="0" radius="2.54" width="0.1524" layer="21"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" diameter="1.778" shape="long" rot="R90"/>
<pad name="K" x="1.27" y="0" drill="0.8128" diameter="1.778" shape="long" rot="R90"/>
<text x="3.175" y="0.5334" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="3.2004" y="-1.8034" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="1206" urn="urn:adsk.eagle:footprint:15651/1">
<description>&lt;b&gt;CHICAGO MINIATURE LAMP, INC.&lt;/b&gt;&lt;p&gt;
7022X Series SMT LEDs 1206 Package Size</description>
<wire x1="1.55" y1="-0.75" x2="-1.55" y2="-0.75" width="0.1016" layer="51"/>
<wire x1="-1.55" y1="-0.75" x2="-1.55" y2="0.75" width="0.1016" layer="51"/>
<wire x1="-1.55" y1="0.75" x2="1.55" y2="0.75" width="0.1016" layer="51"/>
<wire x1="1.55" y1="0.75" x2="1.55" y2="-0.75" width="0.1016" layer="51"/>
<wire x1="-0.55" y1="-0.5" x2="0.55" y2="-0.5" width="0.1016" layer="21" curve="95.452622"/>
<wire x1="-0.55" y1="-0.5" x2="-0.55" y2="0.5" width="0.1016" layer="51" curve="-84.547378"/>
<wire x1="-0.55" y1="0.5" x2="0.55" y2="0.5" width="0.1016" layer="21" curve="-95.452622"/>
<wire x1="0.55" y1="0.5" x2="0.55" y2="-0.5" width="0.1016" layer="51" curve="-84.547378"/>
<smd name="A" x="-1.422" y="0" dx="1.6" dy="1.803" layer="1"/>
<smd name="C" x="1.422" y="0" dx="1.6" dy="1.803" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.1" y1="-0.1" x2="0.1" y2="0.1" layer="21"/>
<rectangle x1="0.45" y1="-0.7" x2="0.8" y2="-0.45" layer="51"/>
<rectangle x1="0.8" y1="-0.7" x2="0.9" y2="0.5" layer="51"/>
<rectangle x1="0.8" y1="0.55" x2="0.9" y2="0.7" layer="51"/>
<rectangle x1="-0.9" y1="-0.7" x2="-0.8" y2="0.5" layer="51"/>
<rectangle x1="-0.9" y1="0.55" x2="-0.8" y2="0.7" layer="51"/>
<rectangle x1="0.45" y1="-0.7" x2="0.6" y2="-0.45" layer="21"/>
</package>
<package name="LD260" urn="urn:adsk.eagle:footprint:15652/1">
<description>&lt;B&gt;LED&lt;/B&gt;&lt;p&gt;
5 mm, square, Siemens</description>
<wire x1="-1.27" y1="-1.27" x2="0" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0" y1="-1.27" x2="1.27" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.27" x2="0" y2="1.27" width="0.1524" layer="21"/>
<wire x1="0" y1="1.27" x2="-1.27" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.27" x2="1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.27" x2="1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.27" y1="0.889" x2="1.27" y2="0" width="0.1524" layer="51"/>
<wire x1="1.27" y1="0" x2="1.27" y2="-0.889" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.27" x2="-1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-0.889" x2="-1.27" y2="0" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="0.889" width="0.1524" layer="51"/>
<wire x1="0" y1="1.27" x2="0.9917" y2="0.7934" width="0.1524" layer="21" curve="-51.33923"/>
<wire x1="-0.9917" y1="0.7934" x2="0" y2="1.27" width="0.1524" layer="21" curve="-51.33923"/>
<wire x1="0" y1="-1.27" x2="0.9917" y2="-0.7934" width="0.1524" layer="21" curve="51.33923"/>
<wire x1="-0.9917" y1="-0.7934" x2="0" y2="-1.27" width="0.1524" layer="21" curve="51.33923"/>
<wire x1="0.9558" y1="-0.8363" x2="1.27" y2="0" width="0.1524" layer="51" curve="41.185419"/>
<wire x1="0.9756" y1="0.813" x2="1.2699" y2="0" width="0.1524" layer="51" curve="-39.806332"/>
<wire x1="-1.27" y1="0" x2="-0.9643" y2="-0.8265" width="0.1524" layer="51" curve="40.600331"/>
<wire x1="-1.27" y1="0" x2="-0.9643" y2="0.8265" width="0.1524" layer="51" curve="-40.600331"/>
<wire x1="-0.889" y1="0" x2="0" y2="0.889" width="0.1524" layer="51" curve="-90"/>
<wire x1="-0.508" y1="0" x2="0" y2="0.508" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-0.508" x2="0.508" y2="0" width="0.1524" layer="21" curve="90"/>
<wire x1="0" y1="-0.889" x2="0.889" y2="0" width="0.1524" layer="51" curve="90"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-1.2954" y="1.4732" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-2.4892" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="1.27" y1="-0.635" x2="2.032" y2="0.635" layer="51"/>
<rectangle x1="1.905" y1="-0.635" x2="2.032" y2="0.635" layer="21"/>
</package>
<package name="LED2X5" urn="urn:adsk.eagle:footprint:15653/1">
<description>&lt;B&gt;LED&lt;/B&gt;&lt;p&gt;
2 x 5 mm, rectangle</description>
<wire x1="-2.54" y1="-1.27" x2="2.54" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.54" y1="1.27" x2="2.54" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.54" y1="1.27" x2="-2.54" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-1.27" x2="-2.54" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="0" x2="0.508" y2="0" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="0.381" x2="-0.508" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="0.381" x2="0.508" y2="0" width="0.1524" layer="51"/>
<wire x1="0.508" y1="0" x2="1.778" y2="0" width="0.1524" layer="51"/>
<wire x1="0.508" y1="0" x2="-0.508" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="0.508" y1="0.381" x2="0.508" y2="0" width="0.1524" layer="51"/>
<wire x1="0.508" y1="0" x2="0.508" y2="-0.381" width="0.1524" layer="51"/>
<wire x1="0.889" y1="-0.254" x2="1.143" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="1.143" y1="-0.762" x2="1.143" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="1.143" y1="-0.762" x2="0.9398" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="0.9398" y1="-0.6096" x2="1.143" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="1.397" y1="-0.254" x2="1.651" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="1.651" y1="-0.762" x2="1.651" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="1.651" y1="-0.762" x2="1.4478" y2="-0.6096" width="0.1524" layer="51"/>
<wire x1="1.4478" y1="-0.6096" x2="1.651" y2="-0.508" width="0.1524" layer="51"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-2.54" y="1.397" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-2.413" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="2.159" y1="-1.27" x2="2.413" y2="1.27" layer="21"/>
</package>
<package name="LED3MM" urn="urn:adsk.eagle:footprint:15654/1">
<description>&lt;B&gt;LED&lt;/B&gt;&lt;p&gt;
3 mm, round</description>
<wire x1="1.5748" y1="-1.27" x2="1.5748" y2="1.27" width="0.254" layer="51"/>
<wire x1="-1.524" y1="0" x2="-1.1708" y2="0.9756" width="0.1524" layer="51" curve="-39.80361"/>
<wire x1="-1.524" y1="0" x2="-1.1391" y2="-1.0125" width="0.1524" layer="51" curve="41.633208"/>
<wire x1="1.1571" y1="0.9918" x2="1.524" y2="0" width="0.1524" layer="51" curve="-40.601165"/>
<wire x1="1.1708" y1="-0.9756" x2="1.524" y2="0" width="0.1524" layer="51" curve="39.80361"/>
<wire x1="0" y1="1.524" x2="1.2401" y2="0.8858" width="0.1524" layer="21" curve="-54.461337"/>
<wire x1="-1.2192" y1="0.9144" x2="0" y2="1.524" width="0.1524" layer="21" curve="-53.130102"/>
<wire x1="0" y1="-1.524" x2="1.203" y2="-0.9356" width="0.1524" layer="21" curve="52.126876"/>
<wire x1="-1.203" y1="-0.9356" x2="0" y2="-1.524" width="0.1524" layer="21" curve="52.126876"/>
<wire x1="-0.635" y1="0" x2="0" y2="0.635" width="0.1524" layer="51" curve="-90"/>
<wire x1="-1.016" y1="0" x2="0" y2="1.016" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-0.635" x2="0.635" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="0" y1="-1.016" x2="1.016" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="0" y1="2.032" x2="1.561" y2="1.3009" width="0.254" layer="21" curve="-50.193108"/>
<wire x1="-1.7929" y1="0.9562" x2="0" y2="2.032" width="0.254" layer="21" curve="-61.926949"/>
<wire x1="0" y1="-2.032" x2="1.5512" y2="-1.3126" width="0.254" layer="21" curve="49.763022"/>
<wire x1="-1.7643" y1="-1.0082" x2="0" y2="-2.032" width="0.254" layer="21" curve="60.255215"/>
<wire x1="-2.032" y1="0" x2="-1.7891" y2="0.9634" width="0.254" layer="51" curve="-28.301701"/>
<wire x1="-2.032" y1="0" x2="-1.7306" y2="-1.065" width="0.254" layer="51" curve="31.60822"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="1.905" y="0.381" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="1.905" y="-1.651" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="LSU260" urn="urn:adsk.eagle:footprint:15656/1">
<description>&lt;B&gt;LED&lt;/B&gt;&lt;p&gt;
1 mm, round, Siemens</description>
<wire x1="0" y1="-0.508" x2="-1.143" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-1.143" y1="-0.508" x2="-1.143" y2="-0.254" width="0.1524" layer="51"/>
<wire x1="-1.143" y1="0.508" x2="0" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-1.143" y1="-0.254" x2="-1.397" y2="-0.254" width="0.1524" layer="51"/>
<wire x1="-1.143" y1="-0.254" x2="-1.143" y2="0.254" width="0.1524" layer="51"/>
<wire x1="-1.397" y1="-0.254" x2="-1.397" y2="0.254" width="0.1524" layer="51"/>
<wire x1="-1.397" y1="0.254" x2="-1.143" y2="0.254" width="0.1524" layer="51"/>
<wire x1="-1.143" y1="0.254" x2="-1.143" y2="0.508" width="0.1524" layer="51"/>
<wire x1="0.508" y1="-0.254" x2="1.397" y2="-0.254" width="0.1524" layer="51"/>
<wire x1="1.397" y1="-0.254" x2="1.397" y2="0.254" width="0.1524" layer="51"/>
<wire x1="1.397" y1="0.254" x2="0.508" y2="0.254" width="0.1524" layer="51"/>
<wire x1="0.381" y1="-0.381" x2="0.254" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="0.254" y1="-0.508" x2="-0.254" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="-0.381" x2="-0.254" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="0.381" y1="0.381" x2="0.254" y2="0.508" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0.508" x2="-0.254" y2="0.508" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0.381" x2="-0.254" y2="0.508" width="0.1524" layer="21"/>
<wire x1="0" y1="-0.254" x2="0.254" y2="0" width="0.1524" layer="21" curve="90"/>
<wire x1="-0.254" y1="0" x2="0" y2="0.254" width="0.1524" layer="21" curve="-90"/>
<wire x1="0.381" y1="-0.381" x2="0.381" y2="0.381" width="0.1524" layer="21" curve="90"/>
<circle x="0" y="0" radius="0.508" width="0.1524" layer="51"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-1.2954" y="0.8382" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-1.8542" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.397" y1="-0.254" x2="-1.143" y2="0.254" layer="51"/>
<rectangle x1="0.508" y1="-0.254" x2="1.397" y2="0.254" layer="51"/>
</package>
<package name="LZR181" urn="urn:adsk.eagle:footprint:15657/1">
<description>&lt;B&gt;LED BLOCK&lt;/B&gt;&lt;p&gt;
1 LED, Siemens</description>
<wire x1="-1.27" y1="-1.27" x2="1.27" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.27" x2="1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.27" x2="1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.27" y1="0.889" x2="1.27" y2="-0.889" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.27" x2="-1.27" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-0.889" x2="-1.27" y2="0.889" width="0.1524" layer="51"/>
<wire x1="-0.889" y1="0" x2="0" y2="0.889" width="0.1524" layer="51" curve="-90"/>
<wire x1="-0.508" y1="0" x2="0" y2="0.508" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-0.508" x2="0.508" y2="0" width="0.1524" layer="21" curve="90"/>
<wire x1="0" y1="-0.889" x2="0.889" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="-0.8678" y1="0.7439" x2="0" y2="1.143" width="0.1524" layer="21" curve="-49.396139"/>
<wire x1="0" y1="1.143" x2="0.8678" y2="0.7439" width="0.1524" layer="21" curve="-49.396139"/>
<wire x1="-0.8678" y1="-0.7439" x2="0" y2="-1.143" width="0.1524" layer="21" curve="49.396139"/>
<wire x1="0" y1="-1.143" x2="0.8678" y2="-0.7439" width="0.1524" layer="21" curve="49.396139"/>
<wire x1="0.8678" y1="0.7439" x2="1.143" y2="0" width="0.1524" layer="51" curve="-40.604135"/>
<wire x1="0.8678" y1="-0.7439" x2="1.143" y2="0" width="0.1524" layer="51" curve="40.604135"/>
<wire x1="-1.143" y1="0" x2="-0.8678" y2="0.7439" width="0.1524" layer="51" curve="-40.604135"/>
<wire x1="-1.143" y1="0" x2="-0.8678" y2="-0.7439" width="0.1524" layer="51" curve="40.604135"/>
<wire x1="-1.27" y1="1.27" x2="1.27" y2="1.27" width="0.1524" layer="21"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="-1.2954" y="1.4732" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-2.4892" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="1.27" y1="-0.889" x2="1.524" y2="0.254" layer="51"/>
<rectangle x1="-1.524" y1="-0.254" x2="-1.27" y2="0.254" layer="51"/>
</package>
<package name="Q62902-B152" urn="urn:adsk.eagle:footprint:15658/1">
<description>&lt;b&gt;LED HOLDER&lt;/b&gt;&lt;p&gt;
Siemens</description>
<wire x1="-2.9718" y1="-1.8542" x2="-2.9718" y2="-0.254" width="0.1524" layer="21"/>
<wire x1="-2.9718" y1="-0.254" x2="-2.9718" y2="0.254" width="0.1524" layer="21"/>
<wire x1="-2.9718" y1="0.254" x2="-2.9718" y2="1.8542" width="0.1524" layer="21"/>
<wire x1="2.9718" y1="-1.8542" x2="-2.1082" y2="-1.8542" width="0.1524" layer="21"/>
<wire x1="-2.1082" y1="-1.8542" x2="-2.54" y2="-1.8542" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="1.8542" x2="-2.1082" y2="1.8542" width="0.1524" layer="21"/>
<wire x1="-2.1082" y1="1.8542" x2="2.9718" y2="1.8542" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-1.8542" x2="-2.54" y2="1.8542" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-1.8542" x2="-2.9718" y2="-1.8542" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="1.8542" x2="-2.9718" y2="1.8542" width="0.1524" layer="21"/>
<wire x1="-2.9718" y1="0.254" x2="-2.9718" y2="-0.254" width="0.1524" layer="21" curve="180"/>
<wire x1="-1.1486" y1="0.8814" x2="0" y2="1.4478" width="0.1524" layer="21" curve="-52.498642"/>
<wire x1="0" y1="1.4478" x2="1.1476" y2="0.8827" width="0.1524" layer="21" curve="-52.433716"/>
<wire x1="-1.1351" y1="-0.8987" x2="0" y2="-1.4478" width="0.1524" layer="21" curve="51.629985"/>
<wire x1="0" y1="-1.4478" x2="1.1305" y2="-0.9044" width="0.1524" layer="21" curve="51.339172"/>
<wire x1="1.1281" y1="-0.9074" x2="1.4478" y2="0" width="0.1524" layer="51" curve="38.811177"/>
<wire x1="1.1401" y1="0.8923" x2="1.4478" y2="0" width="0.1524" layer="51" curve="-38.048073"/>
<wire x1="-1.4478" y1="0" x2="-1.1305" y2="-0.9044" width="0.1524" layer="51" curve="38.659064"/>
<wire x1="-1.4478" y1="0" x2="-1.1456" y2="0.8853" width="0.1524" layer="51" curve="-37.696376"/>
<wire x1="0" y1="1.7018" x2="1.4674" y2="0.8618" width="0.1524" layer="21" curve="-59.573488"/>
<wire x1="-1.4618" y1="0.8714" x2="0" y2="1.7018" width="0.1524" layer="21" curve="-59.200638"/>
<wire x1="0" y1="-1.7018" x2="1.4571" y2="-0.8793" width="0.1524" layer="21" curve="58.891781"/>
<wire x1="-1.4571" y1="-0.8793" x2="0" y2="-1.7018" width="0.1524" layer="21" curve="58.891781"/>
<wire x1="-1.7018" y1="0" x2="-1.4447" y2="0.8995" width="0.1524" layer="51" curve="-31.907626"/>
<wire x1="-1.7018" y1="0" x2="-1.4502" y2="-0.8905" width="0.1524" layer="51" curve="31.551992"/>
<wire x1="1.4521" y1="0.8874" x2="1.7018" y2="0" width="0.1524" layer="51" curve="-31.429586"/>
<wire x1="1.4459" y1="-0.8975" x2="1.7018" y2="0" width="0.1524" layer="51" curve="31.828757"/>
<wire x1="-2.1082" y1="1.8542" x2="-2.1082" y2="-1.8542" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="0" x2="0" y2="0.635" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.016" y1="0" x2="0" y2="1.016" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-0.635" x2="0.635" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="0.0539" y1="-1.0699" x2="1.0699" y2="-0.0539" width="0.1524" layer="51" curve="90"/>
<wire x1="2.9718" y1="1.8542" x2="2.9718" y2="-1.8542" width="0.1524" layer="21"/>
<pad name="K" x="-1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="A" x="1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="-1.905" y="2.286" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.905" y="-3.556" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="Q62902-B153" urn="urn:adsk.eagle:footprint:15659/1">
<description>&lt;b&gt;LED HOLDER&lt;/b&gt;&lt;p&gt;
Siemens</description>
<wire x1="-5.5118" y1="-3.5052" x2="-5.5118" y2="-0.254" width="0.1524" layer="21"/>
<wire x1="-5.5118" y1="-0.254" x2="-5.5118" y2="0.254" width="0.1524" layer="21"/>
<wire x1="-5.5118" y1="0.254" x2="-5.5118" y2="3.5052" width="0.1524" layer="21"/>
<wire x1="5.5118" y1="-3.5052" x2="-4.6482" y2="-3.5052" width="0.1524" layer="21"/>
<wire x1="-4.6482" y1="-3.5052" x2="-5.08" y2="-3.5052" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="3.5052" x2="-4.6482" y2="3.5052" width="0.1524" layer="21"/>
<wire x1="-4.6482" y1="3.5052" x2="5.5118" y2="3.5052" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-3.5052" x2="-5.08" y2="3.5052" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-3.5052" x2="-5.5118" y2="-3.5052" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="3.5052" x2="-5.5118" y2="3.5052" width="0.1524" layer="21"/>
<wire x1="-5.5118" y1="0.254" x2="-5.5118" y2="-0.254" width="0.1524" layer="21" curve="180"/>
<wire x1="-4.6482" y1="3.5052" x2="-4.6482" y2="-3.5052" width="0.1524" layer="21"/>
<wire x1="5.5118" y1="3.5052" x2="5.5118" y2="-3.5052" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="1.905" x2="-2.54" y2="-1.905" width="0.254" layer="21"/>
<wire x1="0" y1="-1.143" x2="1.143" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="-1.143" y1="0" x2="0" y2="1.143" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-1.651" x2="1.651" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="-1.651" y1="0" x2="0" y2="1.651" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-2.159" x2="2.159" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="-2.2129" y1="0.0539" x2="-0.0539" y2="2.2129" width="0.1524" layer="51" curve="-90.010616"/>
<circle x="0" y="0" radius="2.54" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="3.175" width="0.254" layer="21"/>
<pad name="A" x="1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="K" x="-1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="-4.191" y="3.937" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.318" y="-5.08" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="Q62902-B155" urn="urn:adsk.eagle:footprint:15660/1">
<description>&lt;b&gt;LED HOLDER&lt;/b&gt;&lt;p&gt;
Siemens</description>
<wire x1="-1.27" y1="-3.048" x2="-1.27" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="10.033" y1="3.048" x2="2.921" y2="3.048" width="0.1524" layer="21"/>
<wire x1="10.033" y1="3.048" x2="10.033" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-3.048" x2="2.921" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-3.048" x2="2.921" y2="3.048" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-3.048" x2="10.033" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="2.921" y1="3.048" x2="-1.27" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="2.54" x2="-5.207" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="2.54" x2="-1.27" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="-2.54" x2="-1.27" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-2.54" x2="-1.27" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="2.54" x2="-5.207" y2="-2.54" width="0.1524" layer="21" curve="180"/>
<wire x1="-6.985" y1="0.635" x2="-6.985" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-6.096" y1="1.397" x2="-6.096" y2="-1.397" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="1.905" x2="-5.207" y2="-1.905" width="0.1524" layer="21"/>
<pad name="K" x="7.62" y="1.27" drill="0.8128" shape="long"/>
<pad name="A" x="7.62" y="-1.27" drill="0.8128" shape="long"/>
<text x="3.302" y="-2.794" size="1.016" layer="21" ratio="14">A+</text>
<text x="3.302" y="1.778" size="1.016" layer="21" ratio="14">K-</text>
<text x="11.684" y="-2.794" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="0.635" y="-4.445" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="2.921" y1="1.016" x2="6.731" y2="1.524" layer="21"/>
<rectangle x1="2.921" y1="-1.524" x2="6.731" y2="-1.016" layer="21"/>
<hole x="0" y="0" drill="0.8128"/>
</package>
<package name="Q62902-B156" urn="urn:adsk.eagle:footprint:15661/1">
<description>&lt;b&gt;LED HOLDER&lt;/b&gt;&lt;p&gt;
Siemens</description>
<wire x1="2.54" y1="-1.905" x2="2.54" y2="1.905" width="0.1524" layer="21"/>
<wire x1="-1.143" y1="0" x2="0" y2="1.143" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-1.143" x2="1.143" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="-1.651" y1="0" x2="0" y2="1.651" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-1.651" x2="1.651" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="-2.159" y1="0" x2="0" y2="2.159" width="0.1524" layer="51" curve="-90"/>
<wire x1="0.0539" y1="-2.2129" x2="2.2129" y2="-0.0539" width="0.1524" layer="51" curve="90.005308"/>
<wire x1="2.54" y1="3.81" x2="3.81" y2="2.54" width="0.1524" layer="21"/>
<wire x1="2.54" y1="3.81" x2="-3.81" y2="3.81" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-3.81" x2="-3.81" y2="3.81" width="0.1524" layer="21"/>
<wire x1="3.81" y1="2.54" x2="3.81" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-3.81" x2="-2.54" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-3.302" x2="-2.54" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-3.81" x2="2.54" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-3.302" x2="2.54" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-3.302" x2="-2.54" y2="-3.302" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="2.54" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="3.175" width="0.254" layer="21"/>
<pad name="A" x="-1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="K" x="1.27" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-3.81" y="4.0894" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.7846" y="-5.3594" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-3.556" y="-3.302" size="1.016" layer="21" ratio="14">+</text>
<text x="2.794" y="-3.302" size="1.016" layer="21" ratio="14">-</text>
</package>
<package name="SFH480" urn="urn:adsk.eagle:footprint:15662/1">
<description>&lt;B&gt;IR LED&lt;/B&gt;&lt;p&gt;
infrared emitting diode, Infineon
TO-18, lead spacing 2.54 mm, cathode marking&lt;p&gt;
Inifineon</description>
<wire x1="-2.159" y1="1.524" x2="-2.794" y2="2.159" width="0.1524" layer="21"/>
<wire x1="-2.794" y1="2.159" x2="-2.159" y2="2.794" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="2.159" x2="-2.159" y2="2.794" width="0.1524" layer="21"/>
<wire x1="0" y1="1.778" x2="1.5358" y2="0.8959" width="0.1524" layer="21" curve="-59.743278"/>
<wire x1="-1.5358" y1="0.8959" x2="0" y2="1.778" width="0.1524" layer="21" curve="-59.743278"/>
<wire x1="-1.5358" y1="-0.8959" x2="0" y2="-1.778" width="0.1524" layer="21" curve="59.743278"/>
<wire x1="0" y1="-1.778" x2="1.5358" y2="-0.8959" width="0.1524" layer="21" curve="59.743278"/>
<wire x1="1.5142" y1="0.9318" x2="1.778" y2="0" width="0.1524" layer="51" curve="-31.606487"/>
<wire x1="1.5" y1="-0.9546" x2="1.778" y2="0" width="0.1524" layer="51" curve="32.472615"/>
<wire x1="-1.778" y1="0" x2="-1.5142" y2="-0.9318" width="0.1524" layer="51" curve="31.606487"/>
<wire x1="-1.778" y1="0" x2="-1.5" y2="0.9546" width="0.1524" layer="51" curve="-32.472615"/>
<wire x1="-0.635" y1="0" x2="0" y2="0.635" width="0.1524" layer="51" curve="-90"/>
<wire x1="-1.016" y1="0" x2="0" y2="1.016" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-0.635" x2="0.635" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="0.0539" y1="-1.0699" x2="1.0699" y2="-0.0539" width="0.1524" layer="51" curve="90"/>
<circle x="0" y="0" radius="2.667" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="2.413" width="0.254" layer="21"/>
<pad name="K" x="-1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="A" x="1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="-1.27" y="3.048" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-4.318" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="SFH482" urn="urn:adsk.eagle:footprint:15650/1">
<description>&lt;B&gt;IR LED&lt;/B&gt;&lt;p&gt;
infrared emitting diode, Infineon
TO-18, lead spacing 2.54 mm, cathode marking&lt;p&gt;
Inifineon</description>
<wire x1="-2.159" y1="1.524" x2="-2.794" y2="2.159" width="0.1524" layer="21"/>
<wire x1="-2.794" y1="2.159" x2="-2.159" y2="2.794" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="2.159" x2="-2.159" y2="2.794" width="0.1524" layer="21"/>
<wire x1="0" y1="1.778" x2="1.5358" y2="0.8959" width="0.1524" layer="21" curve="-59.743278"/>
<wire x1="-1.5358" y1="0.8959" x2="0" y2="1.778" width="0.1524" layer="21" curve="-59.743278"/>
<wire x1="-1.5358" y1="-0.8959" x2="0" y2="-1.778" width="0.1524" layer="21" curve="59.743278"/>
<wire x1="0" y1="-1.778" x2="1.5358" y2="-0.8959" width="0.1524" layer="21" curve="59.743278"/>
<wire x1="1.5142" y1="0.9318" x2="1.778" y2="0" width="0.1524" layer="51" curve="-31.606487"/>
<wire x1="1.5" y1="-0.9546" x2="1.778" y2="0" width="0.1524" layer="51" curve="32.472615"/>
<wire x1="-1.778" y1="0" x2="-1.5142" y2="-0.9318" width="0.1524" layer="51" curve="31.606487"/>
<wire x1="-1.778" y1="0" x2="-1.5" y2="0.9546" width="0.1524" layer="51" curve="-32.472615"/>
<wire x1="-0.635" y1="0" x2="0" y2="0.635" width="0.1524" layer="51" curve="-90"/>
<wire x1="-1.016" y1="0" x2="0" y2="1.016" width="0.1524" layer="51" curve="-90"/>
<wire x1="0" y1="-0.635" x2="0.635" y2="0" width="0.1524" layer="51" curve="90"/>
<wire x1="0.0539" y1="-1.0699" x2="1.0699" y2="-0.0539" width="0.1524" layer="51" curve="90"/>
<circle x="0" y="0" radius="2.667" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="2.413" width="0.254" layer="21"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="-1.27" y="3.048" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-4.318" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="U57X32" urn="urn:adsk.eagle:footprint:15640/1">
<description>&lt;B&gt;LED&lt;/B&gt;&lt;p&gt;
rectangle, 5.7 x 3.2 mm</description>
<wire x1="-3.175" y1="1.905" x2="3.175" y2="1.905" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-1.905" x2="3.175" y2="1.905" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-1.905" x2="-3.175" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="1.905" x2="-3.175" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-2.667" y1="1.397" x2="2.667" y2="1.397" width="0.1524" layer="21"/>
<wire x1="2.667" y1="-1.397" x2="2.667" y2="1.397" width="0.1524" layer="21"/>
<wire x1="2.667" y1="-1.397" x2="-2.667" y2="-1.397" width="0.1524" layer="21"/>
<wire x1="-2.667" y1="1.397" x2="-2.667" y2="-1.397" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="1.016" x2="2.54" y2="1.016" width="0.1524" layer="51"/>
<wire x1="2.286" y1="1.27" x2="2.286" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="-2.54" y1="0.508" x2="2.54" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-2.54" y1="0" x2="2.54" y2="0" width="0.1524" layer="51"/>
<wire x1="-2.54" y1="-0.508" x2="2.54" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-2.54" y1="-1.016" x2="2.54" y2="-1.016" width="0.1524" layer="51"/>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="1.27" x2="-1.778" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="-0.762" y1="1.27" x2="-0.762" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="-0.254" y1="1.27" x2="-0.254" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="0.254" y1="1.27" x2="0.254" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="0.762" y1="1.27" x2="0.762" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="1.27" y1="1.27" x2="1.27" y2="-1.27" width="0.1524" layer="51"/>
<wire x1="1.778" y1="1.27" x2="1.778" y2="-1.27" width="0.1524" layer="51"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="K" x="1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="3.683" y="0.254" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="3.683" y="-1.524" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="IRL80A" urn="urn:adsk.eagle:footprint:15663/1">
<description>&lt;B&gt;IR LED&lt;/B&gt;&lt;p&gt;
IR transmitter Siemens</description>
<wire x1="0.889" y1="2.286" x2="0.889" y2="1.778" width="0.1524" layer="21"/>
<wire x1="0.889" y1="1.778" x2="0.889" y2="0.762" width="0.1524" layer="51"/>
<wire x1="0.889" y1="0.762" x2="0.889" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="0.889" y1="-0.635" x2="0.889" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="0.889" y1="-1.778" x2="0.889" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="0.889" y1="-2.286" x2="-0.889" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-0.889" y1="2.286" x2="-0.889" y2="1.778" width="0.1524" layer="21"/>
<wire x1="-0.889" y1="1.778" x2="-0.889" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-0.889" y1="0.762" x2="-0.889" y2="-0.762" width="0.1524" layer="21"/>
<wire x1="-0.889" y1="-0.762" x2="-0.889" y2="-1.778" width="0.1524" layer="51"/>
<wire x1="-0.889" y1="-1.778" x2="-0.889" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-0.889" y1="2.286" x2="0.889" y2="2.286" width="0.1524" layer="21"/>
<wire x1="-0.889" y1="-0.762" x2="-0.889" y2="0.762" width="0.1524" layer="21" curve="-180"/>
<wire x1="-1.397" y1="0.254" x2="-1.397" y2="-0.254" width="0.0508" layer="21"/>
<wire x1="-1.143" y1="0.508" x2="-1.143" y2="-0.508" width="0.0508" layer="21"/>
<pad name="K" x="0" y="1.27" drill="0.8128" shape="octagon"/>
<pad name="A" x="0" y="-1.27" drill="0.8128" shape="octagon"/>
<text x="1.27" y="0.381" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="1.27" y="-1.651" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="P-LCC-2" urn="urn:adsk.eagle:footprint:15664/1">
<description>&lt;b&gt;TOPLED® High-optical Power LED (HOP)&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... ls_t675.pdf</description>
<wire x1="-1.4" y1="-1.05" x2="-1.4" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-1.4" y1="-1.6" x2="-1.1" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-1.1" y1="-1.6" x2="-0.85" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-0.85" y1="-1.6" x2="1.1" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="1.1" y1="-1.6" x2="1.4" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="1.4" y1="-1.6" x2="1.4" y2="1.6" width="0.2032" layer="51"/>
<wire x1="1.4" y1="1.6" x2="1.1" y2="1.6" width="0.2032" layer="51"/>
<wire x1="1.1" y1="1.6" x2="-1.1" y2="1.6" width="0.2032" layer="51"/>
<wire x1="-1.1" y1="1.6" x2="-1.4" y2="1.6" width="0.2032" layer="51"/>
<wire x1="-1.1" y1="1.6" x2="-1.1" y2="1.8" width="0.1016" layer="51"/>
<wire x1="-1.1" y1="1.8" x2="1.1" y2="1.8" width="0.1016" layer="51"/>
<wire x1="1.1" y1="1.8" x2="1.1" y2="1.6" width="0.1016" layer="51"/>
<wire x1="-1.1" y1="-1.6" x2="-1.1" y2="-1.8" width="0.1016" layer="51"/>
<wire x1="-1.1" y1="-1.8" x2="1.1" y2="-1.8" width="0.1016" layer="51"/>
<wire x1="1.1" y1="-1.8" x2="1.1" y2="-1.6" width="0.1016" layer="51"/>
<wire x1="-0.85" y1="-1.6" x2="-1.4" y2="-1.05" width="0.2032" layer="51"/>
<wire x1="-1.4" y1="1.6" x2="-1.4" y2="-1.05" width="0.2032" layer="51"/>
<circle x="0" y="0" radius="1.1" width="0.2032" layer="51"/>
<smd name="C" x="0" y="-2.75" dx="4" dy="4" layer="1" stop="no" cream="no"/>
<smd name="A" x="0" y="2.75" dx="4" dy="4" layer="1" stop="no" cream="no"/>
<text x="-2.54" y="-1.905" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="3.81" y="-1.905" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<text x="-0.635" y="2.54" size="1.27" layer="21">A</text>
<text x="-0.635" y="-3.81" size="1.27" layer="21">C</text>
<rectangle x1="-1.3" y1="-2.25" x2="1.3" y2="-0.75" layer="31"/>
<rectangle x1="-1.3" y1="0.75" x2="1.3" y2="2.25" layer="31"/>
<rectangle x1="-0.25" y1="-0.25" x2="0.25" y2="0.25" layer="21"/>
<rectangle x1="-1.4" y1="0.65" x2="1.4" y2="2.35" layer="29"/>
<rectangle x1="-1.4" y1="-2.35" x2="1.4" y2="-0.65" layer="29"/>
</package>
<package name="OSRAM-MINI-TOP-LED" urn="urn:adsk.eagle:footprint:15665/1">
<description>&lt;b&gt;BLUE LINETM Hyper Mini TOPLED® Hyper-Bright LED&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LB M676.pdf</description>
<wire x1="-0.6" y1="0.9" x2="-0.6" y2="-0.7" width="0.1016" layer="51"/>
<wire x1="-0.45" y1="-0.9" x2="-0.4" y2="-0.9" width="0.1016" layer="51"/>
<wire x1="-0.4" y1="-0.9" x2="0.6" y2="-0.9" width="0.1016" layer="51"/>
<wire x1="0.6" y1="-0.9" x2="0.6" y2="0.9" width="0.1016" layer="51"/>
<wire x1="0.6" y1="0.9" x2="-0.6" y2="0.9" width="0.1016" layer="51"/>
<wire x1="-0.45" y1="0.95" x2="-0.45" y2="1.1" width="0.1016" layer="51"/>
<wire x1="-0.45" y1="1.1" x2="0.45" y2="1.1" width="0.1016" layer="51"/>
<wire x1="0.45" y1="1.1" x2="0.45" y2="0.95" width="0.1016" layer="51"/>
<wire x1="-0.6" y1="-0.7" x2="-0.4" y2="-0.9" width="0.1016" layer="51"/>
<wire x1="-0.45" y1="-0.9" x2="-0.45" y2="-1.1" width="0.1016" layer="51"/>
<wire x1="-0.45" y1="-1.1" x2="0.45" y2="-1.1" width="0.1016" layer="51"/>
<wire x1="0.45" y1="-1.1" x2="0.45" y2="-0.95" width="0.1016" layer="51"/>
<smd name="A" x="0" y="2.6" dx="4" dy="4" layer="1" stop="no" cream="no"/>
<smd name="C" x="0" y="-2.6" dx="4" dy="4" layer="1" stop="no" cream="no"/>
<text x="-0.635" y="1.905" size="1.27" layer="21">A</text>
<text x="-0.635" y="-3.175" size="1.27" layer="21">C</text>
<text x="-2.54" y="-1.905" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="3.81" y="-1.905" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.5" y1="0.6" x2="0.5" y2="1.4" layer="29"/>
<rectangle x1="-0.5" y1="-1.4" x2="0.5" y2="-0.6" layer="29"/>
<rectangle x1="-0.15" y1="-0.6" x2="0.15" y2="-0.3" layer="51"/>
<rectangle x1="-0.45" y1="0.65" x2="0.45" y2="1.35" layer="31"/>
<rectangle x1="-0.45" y1="-1.35" x2="0.45" y2="-0.65" layer="31"/>
</package>
<package name="OSRAM-SIDELED" urn="urn:adsk.eagle:footprint:15666/1">
<description>&lt;b&gt;Super SIDELED® High-Current LED&lt;/b&gt;&lt;p&gt;
LG A672, LP A672 &lt;br&gt;
Source: http://www.osram.convergy.de/ ... LG_LP_A672.pdf (2004.05.13)</description>
<wire x1="-1.85" y1="-2.05" x2="-1.85" y2="-0.75" width="0.1016" layer="51"/>
<wire x1="-1.85" y1="-0.75" x2="-1.7" y2="-0.75" width="0.1016" layer="51"/>
<wire x1="-1.7" y1="-0.75" x2="-1.7" y2="0.75" width="0.1016" layer="51"/>
<wire x1="-1.7" y1="0.75" x2="-1.85" y2="0.75" width="0.1016" layer="51"/>
<wire x1="-1.85" y1="0.75" x2="-1.85" y2="2.05" width="0.1016" layer="51"/>
<wire x1="-1.85" y1="2.05" x2="0.9" y2="2.05" width="0.1016" layer="51"/>
<wire x1="0.9" y1="2.05" x2="0.9" y2="-2.05" width="0.1016" layer="51"/>
<wire x1="0.9" y1="-2.05" x2="-1.85" y2="-2.05" width="0.1016" layer="51"/>
<wire x1="0.9" y1="-2.05" x2="1.05" y2="-2.05" width="0.1016" layer="51"/>
<wire x1="1.05" y1="-2.05" x2="1.85" y2="-1.85" width="0.1016" layer="51"/>
<wire x1="1.85" y1="-1.85" x2="1.85" y2="1.85" width="0.1016" layer="51"/>
<wire x1="1.85" y1="1.85" x2="1.05" y2="2.05" width="0.1016" layer="51"/>
<wire x1="1.05" y1="2.05" x2="0.9" y2="2.05" width="0.1016" layer="51"/>
<wire x1="1.05" y1="2.05" x2="1.05" y2="-2.05" width="0.1016" layer="51"/>
<wire x1="-0.55" y1="-0.9" x2="-0.55" y2="0.9" width="0.1016" layer="51" curve="-167.319617"/>
<wire x1="-0.55" y1="-0.9" x2="0.85" y2="-1.2" width="0.1016" layer="51" style="shortdash"/>
<wire x1="-0.55" y1="0.9" x2="0.85" y2="1.2" width="0.1016" layer="51" style="shortdash"/>
<smd name="C" x="0" y="-2.5" dx="4" dy="4" layer="1" stop="no" cream="no"/>
<smd name="A" x="0" y="2.5" dx="4" dy="4" layer="1" stop="no" cream="no"/>
<text x="0.635" y="-3.175" size="1.27" layer="21" rot="R90">C</text>
<text x="0.635" y="2.54" size="1.27" layer="21" rot="R90">A</text>
<text x="-2.54" y="-2.54" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="3.81" y="-2.54" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-2.1" y1="-2.2" x2="2.1" y2="-0.4" layer="29"/>
<rectangle x1="-2.1" y1="0.4" x2="2.1" y2="2.2" layer="29"/>
<rectangle x1="-1.9" y1="-2.1" x2="1.9" y2="-0.6" layer="31"/>
<rectangle x1="-1.9" y1="0.6" x2="1.9" y2="2.1" layer="31"/>
<rectangle x1="-1.85" y1="-2.05" x2="-0.7" y2="-1" layer="51"/>
</package>
<package name="SMART-LED" urn="urn:adsk.eagle:footprint:15667/1">
<description>&lt;b&gt;SmartLEDTM Hyper-Bright LED&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LA_LO_LS_LY L896.pdf</description>
<wire x1="-0.35" y1="0.6" x2="0.35" y2="0.6" width="0.1016" layer="51" style="shortdash"/>
<wire x1="0.35" y1="0.6" x2="0.35" y2="-0.6" width="0.1016" layer="21" style="shortdash"/>
<wire x1="0.35" y1="-0.6" x2="0.15" y2="-0.6" width="0.1016" layer="51" style="shortdash"/>
<wire x1="0.15" y1="-0.6" x2="-0.35" y2="-0.6" width="0.1016" layer="51" style="shortdash"/>
<wire x1="-0.35" y1="-0.6" x2="-0.35" y2="0.6" width="0.1016" layer="21" style="shortdash"/>
<wire x1="0.35" y1="-0.4" x2="0.15" y2="-0.6" width="0.1016" layer="51" style="shortdash"/>
<smd name="A" x="0" y="0.725" dx="0.35" dy="0.35" layer="1"/>
<smd name="B" x="0" y="-0.725" dx="0.35" dy="0.35" layer="1"/>
<text x="-0.635" y="-0.635" size="1.016" layer="25" rot="R90">&gt;NAME</text>
<text x="1.905" y="-0.635" size="1.016" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.15" y1="-0.35" x2="0.15" y2="-0.05" layer="21"/>
<rectangle x1="-0.15" y1="0.6" x2="0.15" y2="0.85" layer="51"/>
<rectangle x1="-0.15" y1="-0.85" x2="0.15" y2="-0.6" layer="51"/>
</package>
<package name="P-LCC-2-TOPLED-RG" urn="urn:adsk.eagle:footprint:15668/1">
<description>&lt;b&gt;Hyper TOPLED® RG Hyper-Bright LED&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LA_LO_LS_LY T776.pdf</description>
<wire x1="-1.4" y1="-1.05" x2="-1.4" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-1.4" y1="-1.6" x2="-1.1" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-1.1" y1="-1.6" x2="-0.85" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-0.85" y1="-1.6" x2="1.1" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="1.1" y1="-1.6" x2="1.4" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="1.4" y1="-1.6" x2="1.4" y2="1.6" width="0.2032" layer="51"/>
<wire x1="1.4" y1="1.6" x2="1.1" y2="1.6" width="0.2032" layer="51"/>
<wire x1="1.1" y1="1.6" x2="-1.1" y2="1.6" width="0.2032" layer="51"/>
<wire x1="-1.1" y1="1.6" x2="-1.4" y2="1.6" width="0.2032" layer="51"/>
<wire x1="-1.1" y1="1.6" x2="-1.1" y2="2.45" width="0.1016" layer="51"/>
<wire x1="1.1" y1="2.45" x2="1.1" y2="1.6" width="0.1016" layer="51"/>
<wire x1="-1.1" y1="-1.6" x2="-1.1" y2="-2.45" width="0.1016" layer="51"/>
<wire x1="1.1" y1="-2.45" x2="1.1" y2="-1.6" width="0.1016" layer="51"/>
<wire x1="-0.85" y1="-1.6" x2="-1.4" y2="-1.05" width="0.2032" layer="51"/>
<wire x1="-1.4" y1="1.6" x2="-1.4" y2="-1.05" width="0.2032" layer="51"/>
<circle x="0" y="0" radius="1.1" width="0.2032" layer="21"/>
<smd name="C" x="0" y="-3.5" dx="4" dy="4" layer="1" stop="no" cream="no"/>
<smd name="A" x="0" y="3.5" dx="4" dy="4" layer="1" stop="no" cream="no"/>
<text x="-2.54" y="-1.905" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="3.81" y="-1.905" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<text x="-0.635" y="3.29" size="1.27" layer="21">A</text>
<text x="-0.635" y="-4.56" size="1.27" layer="21">C</text>
<rectangle x1="-1.3" y1="-3" x2="1.3" y2="-1.5" layer="31"/>
<rectangle x1="-1.3" y1="1.5" x2="1.3" y2="3" layer="31"/>
<rectangle x1="-0.25" y1="-0.25" x2="0.25" y2="0.25" layer="21"/>
<rectangle x1="-1.15" y1="2.4" x2="1.15" y2="2.7" layer="51"/>
<rectangle x1="-1.15" y1="-2.7" x2="1.15" y2="-2.4" layer="51"/>
<rectangle x1="-1.5" y1="1.5" x2="1.5" y2="3.2" layer="29"/>
<rectangle x1="-1.5" y1="-3.2" x2="1.5" y2="-1.5" layer="29"/>
<hole x="0" y="0" drill="2.8"/>
</package>
<package name="MICRO-SIDELED" urn="urn:adsk.eagle:footprint:15669/1">
<description>&lt;b&gt;Hyper Micro SIDELED®&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LA_LO_LS_LY Y876.pdf</description>
<wire x1="0.65" y1="1.1" x2="-0.1" y2="1.1" width="0.1016" layer="51"/>
<wire x1="-0.1" y1="1.1" x2="-0.35" y2="1" width="0.1016" layer="51"/>
<wire x1="-0.35" y1="1" x2="-0.35" y2="-0.9" width="0.1016" layer="21"/>
<wire x1="-0.35" y1="-0.9" x2="-0.1" y2="-1.1" width="0.1016" layer="51"/>
<wire x1="-0.1" y1="-1.1" x2="0.65" y2="-1.1" width="0.1016" layer="51"/>
<wire x1="0.65" y1="-1.1" x2="0.65" y2="1.1" width="0.1016" layer="21"/>
<wire x1="0.6" y1="0.9" x2="0.25" y2="0.7" width="0.0508" layer="21"/>
<wire x1="0.25" y1="0.7" x2="0.25" y2="-0.7" width="0.0508" layer="21"/>
<wire x1="0.25" y1="-0.7" x2="0.6" y2="-0.9" width="0.0508" layer="21"/>
<smd name="A" x="0" y="1.95" dx="1.6" dy="1.6" layer="1" stop="no" cream="no"/>
<smd name="C" x="0" y="-1.95" dx="1.6" dy="1.6" layer="1" stop="no" cream="no"/>
<text x="-1.27" y="-1.905" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="2.54" y="-1.905" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.4" y1="1.1" x2="0.4" y2="1.8" layer="29"/>
<rectangle x1="-0.4" y1="-1.8" x2="0.4" y2="-1.1" layer="29"/>
<rectangle x1="-0.35" y1="-1.75" x2="0.35" y2="-1.15" layer="31"/>
<rectangle x1="-0.35" y1="1.15" x2="0.35" y2="1.75" layer="31"/>
<rectangle x1="-0.125" y1="1.125" x2="0.125" y2="1.75" layer="51"/>
<rectangle x1="-0.125" y1="-1.75" x2="0.125" y2="-1.125" layer="51"/>
</package>
<package name="P-LCC-4" urn="urn:adsk.eagle:footprint:15670/1">
<description>&lt;b&gt;Power TOPLED®&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LA_LO_LA_LY E67B.pdf</description>
<wire x1="-1.4" y1="-1.05" x2="-1.4" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-1.4" y1="-1.6" x2="-1" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-1" y1="-1.6" x2="-0.85" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="-0.85" y1="-1.6" x2="1" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="1" y1="-1.6" x2="1.4" y2="-1.6" width="0.2032" layer="51"/>
<wire x1="1.4" y1="-1.6" x2="1.4" y2="1.6" width="0.2032" layer="51"/>
<wire x1="1.4" y1="1.6" x2="1.1" y2="1.6" width="0.2032" layer="51"/>
<wire x1="1.1" y1="1.6" x2="-1" y2="1.6" width="0.2032" layer="51"/>
<wire x1="-1" y1="1.6" x2="-1.4" y2="1.6" width="0.2032" layer="51"/>
<wire x1="-1" y1="1.6" x2="-1" y2="1.8" width="0.1016" layer="51"/>
<wire x1="-1" y1="1.8" x2="-0.5" y2="1.8" width="0.1016" layer="51"/>
<wire x1="-0.5" y1="1.8" x2="-0.5" y2="1.65" width="0.1016" layer="51"/>
<wire x1="0.5" y1="1.65" x2="0.5" y2="1.8" width="0.1016" layer="51"/>
<wire x1="0.5" y1="1.8" x2="1.1" y2="1.8" width="0.1016" layer="51"/>
<wire x1="1.1" y1="1.8" x2="1.1" y2="1.6" width="0.1016" layer="51"/>
<wire x1="-1" y1="-1.6" x2="-1" y2="-1.8" width="0.1016" layer="51"/>
<wire x1="-1" y1="-1.8" x2="-0.5" y2="-1.8" width="0.1016" layer="51"/>
<wire x1="-0.5" y1="-1.8" x2="-0.5" y2="-1.65" width="0.1016" layer="51"/>
<wire x1="0.5" y1="-1.65" x2="0.5" y2="-1.8" width="0.1016" layer="51"/>
<wire x1="0.5" y1="-1.8" x2="1" y2="-1.8" width="0.1016" layer="51"/>
<wire x1="1" y1="-1.8" x2="1" y2="-1.6" width="0.1016" layer="51"/>
<wire x1="-0.85" y1="-1.6" x2="-1.4" y2="-1.05" width="0.2032" layer="51"/>
<wire x1="-1.4" y1="1.6" x2="-1.4" y2="-1.05" width="0.2032" layer="51"/>
<circle x="0" y="0" radius="1.1" width="0.2032" layer="51"/>
<smd name="A" x="-2" y="3.15" dx="3.3" dy="4.8" layer="1" stop="no" cream="no"/>
<smd name="C@3" x="2" y="3.15" dx="3.3" dy="4.8" layer="1" stop="no" cream="no"/>
<smd name="C@4" x="2" y="-3.15" dx="3.3" dy="4.8" layer="1" stop="no" cream="no"/>
<smd name="C@1" x="-2" y="-3.15" dx="3.3" dy="4.8" layer="1" stop="no" cream="no"/>
<text x="-3.81" y="-2.54" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="5.08" y="-2.54" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<text x="-1.905" y="-3.81" size="1.27" layer="21">C</text>
<text x="-1.905" y="2.54" size="1.27" layer="21">A</text>
<text x="1.27" y="2.54" size="1.27" layer="21">C</text>
<text x="1.27" y="-3.81" size="1.27" layer="21">C</text>
<rectangle x1="-1.15" y1="0.75" x2="-0.35" y2="1.85" layer="29"/>
<rectangle x1="0.35" y1="0.75" x2="1.15" y2="1.85" layer="29"/>
<rectangle x1="0.35" y1="-1.85" x2="1.15" y2="-0.75" layer="29"/>
<rectangle x1="-1.15" y1="-1.85" x2="-0.35" y2="-0.75" layer="29"/>
<rectangle x1="-1.1" y1="-1.8" x2="-0.4" y2="-0.8" layer="31"/>
<rectangle x1="0.4" y1="-1.8" x2="1.1" y2="-0.8" layer="31"/>
<rectangle x1="0.4" y1="0.8" x2="1.1" y2="1.8" layer="31"/>
<rectangle x1="-1.1" y1="0.8" x2="-0.4" y2="1.8" layer="31"/>
<rectangle x1="-0.2" y1="-0.2" x2="0.2" y2="0.2" layer="21"/>
</package>
<package name="CHIP-LED0603" urn="urn:adsk.eagle:footprint:15671/1">
<description>&lt;b&gt;Hyper CHIPLED Hyper-Bright LED&lt;/b&gt;&lt;p&gt;
LB Q993&lt;br&gt;
Source: http://www.osram.convergy.de/ ... Lb_q993.pdf</description>
<wire x1="-0.4" y1="0.45" x2="-0.4" y2="-0.45" width="0.1016" layer="51"/>
<wire x1="0.4" y1="0.45" x2="0.4" y2="-0.45" width="0.1016" layer="51"/>
<smd name="C" x="0" y="0.75" dx="0.8" dy="0.8" layer="1"/>
<smd name="A" x="0" y="-0.75" dx="0.8" dy="0.8" layer="1"/>
<text x="-0.635" y="-0.635" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="1.905" y="-0.635" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.45" y1="0.45" x2="0.45" y2="0.85" layer="51"/>
<rectangle x1="-0.45" y1="-0.85" x2="0.45" y2="-0.45" layer="51"/>
<rectangle x1="-0.45" y1="0" x2="-0.3" y2="0.3" layer="21"/>
<rectangle x1="0.3" y1="0" x2="0.45" y2="0.3" layer="21"/>
<rectangle x1="-0.15" y1="0" x2="0.15" y2="0.3" layer="21"/>
</package>
<package name="CHIP-LED0805" urn="urn:adsk.eagle:footprint:15672/1">
<description>&lt;b&gt;Hyper CHIPLED Hyper-Bright LED&lt;/b&gt;&lt;p&gt;
LB R99A&lt;br&gt;
Source: http://www.osram.convergy.de/ ... lb_r99a.pdf</description>
<wire x1="-0.625" y1="0.45" x2="-0.625" y2="-0.45" width="0.1016" layer="51"/>
<wire x1="0.625" y1="0.45" x2="0.625" y2="-0.475" width="0.1016" layer="51"/>
<smd name="C" x="0" y="1.05" dx="1.2" dy="1.2" layer="1"/>
<smd name="A" x="0" y="-1.05" dx="1.2" dy="1.2" layer="1"/>
<text x="-1.27" y="-1.27" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="2.54" y="-1.27" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.675" y1="0" x2="-0.525" y2="0.3" layer="21"/>
<rectangle x1="0.525" y1="0" x2="0.675" y2="0.3" layer="21"/>
<rectangle x1="-0.15" y1="0" x2="0.15" y2="0.3" layer="21"/>
<rectangle x1="-0.675" y1="0.45" x2="0.675" y2="1.05" layer="51"/>
<rectangle x1="-0.675" y1="-1.05" x2="0.675" y2="-0.45" layer="51"/>
</package>
<package name="MINI-TOPLED-SANTANA" urn="urn:adsk.eagle:footprint:15673/1">
<description>&lt;b&gt;Mini TOPLED Santana®&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LG M470.pdf</description>
<wire x1="0.7" y1="-1" x2="0.35" y2="-1" width="0.1016" layer="21"/>
<wire x1="0.35" y1="-1" x2="-0.7" y2="-1" width="0.1016" layer="21"/>
<wire x1="-0.7" y1="-1" x2="-0.7" y2="1" width="0.1016" layer="21"/>
<wire x1="-0.7" y1="1" x2="0.7" y2="1" width="0.1016" layer="21"/>
<wire x1="0.7" y1="1" x2="0.7" y2="-0.65" width="0.1016" layer="21"/>
<wire x1="0.7" y1="-0.65" x2="0.7" y2="-1" width="0.1016" layer="21"/>
<wire x1="0.45" y1="-0.7" x2="-0.45" y2="-0.7" width="0.1016" layer="21"/>
<wire x1="-0.45" y1="-0.7" x2="-0.45" y2="0.7" width="0.1016" layer="21"/>
<wire x1="-0.45" y1="0.7" x2="0.45" y2="0.7" width="0.1016" layer="21"/>
<wire x1="0.45" y1="0.7" x2="0.45" y2="-0.7" width="0.1016" layer="21"/>
<wire x1="0.7" y1="-0.65" x2="0.35" y2="-1" width="0.1016" layer="21"/>
<smd name="C" x="0" y="-2.2" dx="1.6" dy="1.6" layer="1" stop="no" cream="no"/>
<smd name="A" x="0" y="2.2" dx="1.6" dy="1.6" layer="1" stop="no" cream="no"/>
<text x="-1.27" y="-1.905" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="2.54" y="-1.905" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.55" y1="1.5" x2="0.55" y2="2.1" layer="29"/>
<rectangle x1="-0.55" y1="-2.1" x2="0.55" y2="-1.5" layer="29"/>
<rectangle x1="-0.5" y1="-2.05" x2="0.5" y2="-1.55" layer="31"/>
<rectangle x1="-0.5" y1="1.55" x2="0.5" y2="2.05" layer="31"/>
<rectangle x1="-0.2" y1="-0.4" x2="0.15" y2="-0.05" layer="21"/>
<rectangle x1="-0.5" y1="-2.1" x2="0.5" y2="-1.4" layer="51"/>
<rectangle x1="-0.5" y1="1.4" x2="0.5" y2="2.05" layer="51"/>
<rectangle x1="-0.5" y1="1" x2="0.5" y2="1.4" layer="21"/>
<rectangle x1="-0.5" y1="-1.4" x2="0.5" y2="-1.05" layer="21"/>
<hole x="0" y="0" drill="2.7"/>
</package>
<package name="CHIPLED_0805" urn="urn:adsk.eagle:footprint:15674/1">
<description>&lt;b&gt;CHIPLED&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LG_R971.pdf</description>
<wire x1="-0.35" y1="0.925" x2="0.35" y2="0.925" width="0.1016" layer="51" curve="162.394521"/>
<wire x1="-0.35" y1="-0.925" x2="0.35" y2="-0.925" width="0.1016" layer="51" curve="-162.394521"/>
<wire x1="0.575" y1="0.525" x2="0.575" y2="-0.525" width="0.1016" layer="51"/>
<wire x1="-0.575" y1="-0.5" x2="-0.575" y2="0.925" width="0.1016" layer="51"/>
<circle x="-0.45" y="0.85" radius="0.103" width="0.1016" layer="51"/>
<smd name="C" x="0" y="1.05" dx="1.2" dy="1.2" layer="1"/>
<smd name="A" x="0" y="-1.05" dx="1.2" dy="1.2" layer="1"/>
<text x="-1.27" y="-1.27" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="2.54" y="-1.27" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="0.3" y1="0.5" x2="0.625" y2="1" layer="51"/>
<rectangle x1="-0.325" y1="0.5" x2="-0.175" y2="0.75" layer="51"/>
<rectangle x1="0.175" y1="0.5" x2="0.325" y2="0.75" layer="51"/>
<rectangle x1="-0.2" y1="0.5" x2="0.2" y2="0.675" layer="51"/>
<rectangle x1="0.3" y1="-1" x2="0.625" y2="-0.5" layer="51"/>
<rectangle x1="-0.625" y1="-1" x2="-0.3" y2="-0.5" layer="51"/>
<rectangle x1="0.175" y1="-0.75" x2="0.325" y2="-0.5" layer="51"/>
<rectangle x1="-0.325" y1="-0.75" x2="-0.175" y2="-0.5" layer="51"/>
<rectangle x1="-0.2" y1="-0.675" x2="0.2" y2="-0.5" layer="51"/>
<rectangle x1="-0.1" y1="0" x2="0.1" y2="0.2" layer="21"/>
<rectangle x1="-0.6" y1="0.5" x2="-0.3" y2="0.8" layer="51"/>
<rectangle x1="-0.625" y1="0.925" x2="-0.3" y2="1" layer="51"/>
</package>
<package name="CHIPLED_1206" urn="urn:adsk.eagle:footprint:15675/1">
<description>&lt;b&gt;CHIPLED&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LG_LY N971.pdf</description>
<wire x1="-0.4" y1="1.6" x2="0.4" y2="1.6" width="0.1016" layer="51" curve="172.619069"/>
<wire x1="-0.8" y1="-0.95" x2="-0.8" y2="0.95" width="0.1016" layer="51"/>
<wire x1="0.8" y1="0.95" x2="0.8" y2="-0.95" width="0.1016" layer="51"/>
<circle x="-0.55" y="1.425" radius="0.1" width="0.1016" layer="51"/>
<smd name="C" x="0" y="1.75" dx="1.5" dy="1.5" layer="1"/>
<smd name="A" x="0" y="-1.75" dx="1.5" dy="1.5" layer="1"/>
<text x="-1.27" y="-1.27" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="2.54" y="-1.27" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.85" y1="1.525" x2="-0.35" y2="1.65" layer="51"/>
<rectangle x1="-0.85" y1="1.225" x2="-0.625" y2="1.55" layer="51"/>
<rectangle x1="-0.45" y1="1.225" x2="-0.325" y2="1.45" layer="51"/>
<rectangle x1="-0.65" y1="1.225" x2="-0.225" y2="1.35" layer="51"/>
<rectangle x1="0.35" y1="1.3" x2="0.85" y2="1.65" layer="51"/>
<rectangle x1="0.25" y1="1.225" x2="0.85" y2="1.35" layer="51"/>
<rectangle x1="-0.85" y1="0.95" x2="0.85" y2="1.25" layer="51"/>
<rectangle x1="-0.85" y1="-1.65" x2="0.85" y2="-0.95" layer="51"/>
<rectangle x1="-0.85" y1="0.35" x2="-0.525" y2="0.775" layer="21"/>
<rectangle x1="0.525" y1="0.35" x2="0.85" y2="0.775" layer="21"/>
<rectangle x1="-0.175" y1="0" x2="0.175" y2="0.35" layer="21"/>
</package>
<package name="CHIPLED_0603" urn="urn:adsk.eagle:footprint:15676/1">
<description>&lt;b&gt;CHIPLED&lt;/b&gt;&lt;p&gt;
Source: http://www.osram.convergy.de/ ... LG_LY Q971.pdf</description>
<wire x1="-0.3" y1="0.8" x2="0.3" y2="0.8" width="0.1016" layer="51" curve="170.055574"/>
<wire x1="-0.275" y1="-0.825" x2="0.275" y2="-0.825" width="0.0508" layer="51" curve="-180"/>
<wire x1="-0.4" y1="0.375" x2="-0.4" y2="-0.35" width="0.1016" layer="51"/>
<wire x1="0.4" y1="0.35" x2="0.4" y2="-0.35" width="0.1016" layer="51"/>
<circle x="-0.35" y="0.625" radius="0.075" width="0.0508" layer="51"/>
<smd name="C" x="0" y="0.75" dx="0.8" dy="0.8" layer="1"/>
<smd name="A" x="0" y="-0.75" dx="0.8" dy="0.8" layer="1"/>
<text x="-0.635" y="-1.27" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="1.905" y="-1.27" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.45" y1="0.7" x2="-0.25" y2="0.85" layer="51"/>
<rectangle x1="-0.275" y1="0.55" x2="-0.225" y2="0.6" layer="51"/>
<rectangle x1="-0.45" y1="0.35" x2="-0.4" y2="0.725" layer="51"/>
<rectangle x1="0.25" y1="0.55" x2="0.45" y2="0.85" layer="51"/>
<rectangle x1="-0.45" y1="0.35" x2="0.45" y2="0.575" layer="51"/>
<rectangle x1="-0.45" y1="-0.85" x2="-0.25" y2="-0.35" layer="51"/>
<rectangle x1="0.25" y1="-0.85" x2="0.45" y2="-0.35" layer="51"/>
<rectangle x1="-0.275" y1="-0.575" x2="0.275" y2="-0.35" layer="51"/>
<rectangle x1="-0.275" y1="-0.65" x2="-0.175" y2="-0.55" layer="51"/>
<rectangle x1="0.175" y1="-0.65" x2="0.275" y2="-0.55" layer="51"/>
<rectangle x1="-0.125" y1="0" x2="0.125" y2="0.25" layer="21"/>
</package>
<package name="CHIPLED-0603-TTW" urn="urn:adsk.eagle:footprint:15677/1">
<description>&lt;b&gt;CHIPLED-0603&lt;/b&gt;&lt;p&gt;
Recommended Solder Pad useable for SmartLEDTM and Chipled - Package 0603&lt;br&gt;
Package able to withstand TTW-soldering heat&lt;br&gt;
Package suitable for TTW-soldering&lt;br&gt;
Source: http://www.osram.convergy.de/ ... LO_LS_LY L89K.pdf</description>
<wire x1="-0.3" y1="0.8" x2="0.3" y2="0.8" width="0.1016" layer="51" curve="170.055574"/>
<wire x1="-0.275" y1="-0.825" x2="0.275" y2="-0.825" width="0.0508" layer="51" curve="-180"/>
<wire x1="-0.4" y1="0.375" x2="-0.4" y2="-0.35" width="0.1016" layer="51"/>
<wire x1="0.4" y1="0.35" x2="0.4" y2="-0.35" width="0.1016" layer="51"/>
<circle x="-0.35" y="0.625" radius="0.075" width="0.0508" layer="51"/>
<smd name="C" x="0" y="0.875" dx="0.8" dy="0.5" layer="1" stop="no" cream="no"/>
<smd name="A" x="0" y="-0.875" dx="0.8" dy="0.5" layer="1" stop="no" cream="no"/>
<smd name="A@1" x="0" y="-0.5" dx="0.35" dy="0.35" layer="1"/>
<smd name="C@1" x="0" y="0.5" dx="0.35" dy="0.35" layer="1"/>
<text x="-0.635" y="-1.27" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="1.905" y="-1.27" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.45" y1="0.7" x2="-0.25" y2="0.85" layer="51"/>
<rectangle x1="-0.275" y1="0.55" x2="-0.225" y2="0.6" layer="51"/>
<rectangle x1="-0.45" y1="0.35" x2="-0.4" y2="0.725" layer="51"/>
<rectangle x1="0.25" y1="0.55" x2="0.45" y2="0.85" layer="51"/>
<rectangle x1="-0.45" y1="0.35" x2="0.45" y2="0.575" layer="51"/>
<rectangle x1="-0.45" y1="-0.85" x2="-0.25" y2="-0.35" layer="51"/>
<rectangle x1="0.25" y1="-0.85" x2="0.45" y2="-0.35" layer="51"/>
<rectangle x1="-0.275" y1="-0.575" x2="0.275" y2="-0.35" layer="51"/>
<rectangle x1="-0.275" y1="-0.65" x2="-0.175" y2="-0.55" layer="51"/>
<rectangle x1="0.175" y1="-0.65" x2="0.275" y2="-0.55" layer="51"/>
<rectangle x1="-0.125" y1="0" x2="0.125" y2="0.25" layer="21"/>
<rectangle x1="-0.175" y1="0.325" x2="0.175" y2="0.7" layer="29"/>
<rectangle x1="-0.4" y1="0.625" x2="0.4" y2="1.125" layer="29"/>
<rectangle x1="-0.4" y1="-1.125" x2="0.4" y2="-0.625" layer="29"/>
<rectangle x1="-0.175" y1="-0.675" x2="0.175" y2="-0.325" layer="29"/>
</package>
<package name="SMARTLED-TTW" urn="urn:adsk.eagle:footprint:15678/1">
<description>&lt;b&gt;SmartLED TTW&lt;/b&gt;&lt;p&gt;
Recommended Solder Pad useable for SmartLEDTM and Chipled - Package 0603&lt;br&gt;
Package able to withstand TTW-soldering heat&lt;br&gt;
Package suitable for TTW-soldering&lt;br&gt;
Source: http://www.osram.convergy.de/ ... LO_LS_LY L89K.pdf</description>
<wire x1="-0.35" y1="0.6" x2="0.35" y2="0.6" width="0.1016" layer="51" style="shortdash"/>
<wire x1="0.35" y1="0.6" x2="0.35" y2="-0.6" width="0.1016" layer="21" style="shortdash"/>
<wire x1="0.35" y1="-0.6" x2="0.15" y2="-0.6" width="0.1016" layer="51" style="shortdash"/>
<wire x1="0.15" y1="-0.6" x2="-0.35" y2="-0.6" width="0.1016" layer="51" style="shortdash"/>
<wire x1="-0.35" y1="-0.6" x2="-0.35" y2="0.6" width="0.1016" layer="21" style="shortdash"/>
<wire x1="0.35" y1="-0.4" x2="0.15" y2="-0.6" width="0.1016" layer="51" style="shortdash"/>
<smd name="C" x="0" y="0.875" dx="0.8" dy="0.5" layer="1" stop="no" cream="no"/>
<smd name="A" x="0" y="-0.875" dx="0.8" dy="0.5" layer="1" stop="no" cream="no"/>
<smd name="A@1" x="0" y="-0.5" dx="0.35" dy="0.35" layer="1" stop="no" cream="no"/>
<smd name="C@1" x="0" y="0.5" dx="0.35" dy="0.35" layer="1" stop="no" cream="no"/>
<text x="-0.635" y="-1.27" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="1.905" y="-1.27" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-0.175" y1="0.325" x2="0.175" y2="0.7" layer="29"/>
<rectangle x1="-0.15" y1="-0.35" x2="0.15" y2="-0.05" layer="21"/>
<rectangle x1="-0.15" y1="0.6" x2="0.15" y2="0.85" layer="51"/>
<rectangle x1="-0.15" y1="-0.85" x2="0.15" y2="-0.6" layer="51"/>
<rectangle x1="-0.225" y1="0.3" x2="0.225" y2="0.975" layer="31"/>
<rectangle x1="-0.175" y1="-0.7" x2="0.175" y2="-0.325" layer="29" rot="R180"/>
<rectangle x1="-0.225" y1="-0.975" x2="0.225" y2="-0.3" layer="31" rot="R180"/>
</package>
<package name="LUMILED+" urn="urn:adsk.eagle:footprint:15679/1">
<description>&lt;b&gt;Lumileds Lighting. LUXEON®&lt;/b&gt; with cool pad&lt;p&gt;
Source: K2.pdf</description>
<wire x1="-3.575" y1="2.3375" x2="-2.3375" y2="3.575" width="0.2032" layer="21"/>
<wire x1="-2.3375" y1="3.575" x2="2.3375" y2="3.575" width="0.2032" layer="21"/>
<wire x1="3.575" y1="2.3375" x2="3.575" y2="-3.575" width="0.2032" layer="21"/>
<wire x1="3.575" y1="-3.575" x2="-2.3375" y2="-3.575" width="0.2032" layer="21"/>
<wire x1="-2.3375" y1="-3.575" x2="-2.5" y2="-3.4125" width="0.2032" layer="21"/>
<wire x1="-2.5" y1="-3.4125" x2="-3.4125" y2="-2.5" width="0.2032" layer="21" curve="167.429893"/>
<wire x1="-3.4125" y1="-2.5" x2="-3.575" y2="-2.3375" width="0.2032" layer="21"/>
<wire x1="-3.575" y1="-2.3375" x2="-3.575" y2="2.3375" width="0.2032" layer="21"/>
<wire x1="2.3375" y1="3.575" x2="2.5" y2="3.4125" width="0.2032" layer="21"/>
<wire x1="2.5" y1="3.4125" x2="3.4125" y2="2.5" width="0.2032" layer="21" curve="167.429893"/>
<wire x1="3.4125" y1="2.5" x2="3.575" y2="2.3375" width="0.2032" layer="21"/>
<wire x1="-1.725" y1="2.225" x2="-1.0625" y2="2.5625" width="0.2032" layer="21" curve="-255.44999"/>
<wire x1="1.725" y1="-2.225" x2="1.0625" y2="-2.5625" width="0.2032" layer="21" curve="-255.44999"/>
<circle x="0" y="0" radius="2.725" width="0.2032" layer="51"/>
<smd name="1NC" x="-5.2" y="1.15" dx="2.9" dy="1.7" layer="1"/>
<smd name="2+" x="-5.2" y="-1.15" dx="2.9" dy="1.7" layer="1"/>
<smd name="3NC" x="5.2" y="-1.15" dx="2.9" dy="1.7" layer="1" rot="R180"/>
<smd name="4-" x="5.2" y="1.15" dx="2.9" dy="1.7" layer="1" rot="R180"/>
<text x="-3.175" y="3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-5.975" y1="0.575" x2="-3.625" y2="1.6" layer="51"/>
<rectangle x1="-5.975" y1="-1.6" x2="-3.625" y2="-0.575" layer="51"/>
<rectangle x1="3.625" y1="-1.6" x2="5.975" y2="-0.575" layer="51" rot="R180"/>
<rectangle x1="3.625" y1="0.575" x2="5.975" y2="1.6" layer="51" rot="R180"/>
<polygon width="0.4064" layer="1">
<vertex x="2.3383" y="1.35"/>
<vertex x="0" y="2.7"/>
<vertex x="-2.3383" y="1.35"/>
<vertex x="-2.3383" y="-1.35"/>
<vertex x="0" y="-2.7"/>
<vertex x="2.3383" y="-1.35"/>
</polygon>
<polygon width="0.4064" layer="29">
<vertex x="2.3383" y="1.35"/>
<vertex x="0" y="2.7"/>
<vertex x="-2.3383" y="1.35"/>
<vertex x="-2.3383" y="-1.35"/>
<vertex x="0" y="-2.7"/>
<vertex x="2.3383" y="-1.35"/>
</polygon>
<polygon width="0.4064" layer="31">
<vertex x="2.3383" y="1.35"/>
<vertex x="0" y="2.7"/>
<vertex x="-2.3383" y="1.35"/>
<vertex x="-2.3383" y="-1.35"/>
<vertex x="0" y="-2.7"/>
<vertex x="2.3383" y="-1.35"/>
</polygon>
</package>
<package name="LUMILED" urn="urn:adsk.eagle:footprint:15680/1">
<description>&lt;b&gt;Lumileds Lighting. LUXEON®&lt;/b&gt; without cool pad&lt;p&gt;
Source: K2.pdf</description>
<wire x1="-3.575" y1="2.3375" x2="-2.3375" y2="3.575" width="0.2032" layer="21"/>
<wire x1="-2.3375" y1="3.575" x2="2.3375" y2="3.575" width="0.2032" layer="21"/>
<wire x1="3.575" y1="2.3375" x2="3.575" y2="-3.575" width="0.2032" layer="21"/>
<wire x1="3.575" y1="-3.575" x2="-2.3375" y2="-3.575" width="0.2032" layer="21"/>
<wire x1="-2.3375" y1="-3.575" x2="-2.5" y2="-3.4125" width="0.2032" layer="21"/>
<wire x1="-2.5" y1="-3.4125" x2="-3.4125" y2="-2.5" width="0.2032" layer="21" curve="167.429893"/>
<wire x1="-3.4125" y1="-2.5" x2="-3.575" y2="-2.3375" width="0.2032" layer="21"/>
<wire x1="-3.575" y1="-2.3375" x2="-3.575" y2="2.3375" width="0.2032" layer="21"/>
<wire x1="2.3375" y1="3.575" x2="2.5" y2="3.4125" width="0.2032" layer="21"/>
<wire x1="2.5" y1="3.4125" x2="3.4125" y2="2.5" width="0.2032" layer="21" curve="167.429893"/>
<wire x1="3.4125" y1="2.5" x2="3.575" y2="2.3375" width="0.2032" layer="21"/>
<wire x1="-1.725" y1="2.225" x2="-1.0625" y2="2.5625" width="0.2032" layer="21" curve="-255.44999"/>
<wire x1="1.725" y1="-2.225" x2="1.0625" y2="-2.5625" width="0.2032" layer="21" curve="-255.44999"/>
<circle x="0" y="0" radius="2.725" width="0.2032" layer="51"/>
<smd name="1NC" x="-5.2" y="1.15" dx="2.9" dy="1.7" layer="1"/>
<smd name="2+" x="-5.2" y="-1.15" dx="2.9" dy="1.7" layer="1"/>
<smd name="3NC" x="5.2" y="-1.15" dx="2.9" dy="1.7" layer="1" rot="R180"/>
<smd name="4-" x="5.2" y="1.15" dx="2.9" dy="1.7" layer="1" rot="R180"/>
<text x="-3.175" y="3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-5.975" y1="0.575" x2="-3.625" y2="1.6" layer="51"/>
<rectangle x1="-5.975" y1="-1.6" x2="-3.625" y2="-0.575" layer="51"/>
<rectangle x1="3.625" y1="-1.6" x2="5.975" y2="-0.575" layer="51" rot="R180"/>
<rectangle x1="3.625" y1="0.575" x2="5.975" y2="1.6" layer="51" rot="R180"/>
<polygon width="0.4064" layer="29">
<vertex x="2.3383" y="1.35"/>
<vertex x="0" y="2.7"/>
<vertex x="-2.3383" y="1.35"/>
<vertex x="-2.3383" y="-1.35"/>
<vertex x="0" y="-2.7"/>
<vertex x="2.3383" y="-1.35"/>
</polygon>
<polygon width="0.4064" layer="31">
<vertex x="2.3383" y="1.35"/>
<vertex x="0" y="2.7"/>
<vertex x="-2.3383" y="1.35"/>
<vertex x="-2.3383" y="-1.35"/>
<vertex x="0" y="-2.7"/>
<vertex x="2.3383" y="-1.35"/>
</polygon>
</package>
<package name="LED10MM" urn="urn:adsk.eagle:footprint:15681/1">
<description>&lt;B&gt;LED&lt;/B&gt;&lt;p&gt;
10 mm, round</description>
<wire x1="5.08" y1="-2.54" x2="5.08" y2="2.54" width="0.254" layer="21" curve="-306.869898"/>
<wire x1="4.445" y1="0" x2="0" y2="-4.445" width="0.127" layer="21" curve="-90"/>
<wire x1="3.81" y1="0" x2="0" y2="-3.81" width="0.127" layer="21" curve="-90"/>
<wire x1="3.175" y1="0" x2="0" y2="-3.175" width="0.127" layer="21" curve="-90"/>
<wire x1="2.54" y1="0" x2="0" y2="-2.54" width="0.127" layer="21" curve="-90"/>
<wire x1="-4.445" y1="0" x2="0" y2="4.445" width="0.127" layer="21" curve="-90"/>
<wire x1="-3.81" y1="0" x2="0" y2="3.81" width="0.127" layer="21" curve="-90"/>
<wire x1="-3.175" y1="0" x2="0" y2="3.175" width="0.127" layer="21" curve="-90"/>
<wire x1="-2.54" y1="0" x2="0" y2="2.54" width="0.127" layer="21" curve="-90"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-2.54" width="0.254" layer="21"/>
<circle x="0" y="0" radius="5.08" width="0.127" layer="21"/>
<pad name="K" x="1.27" y="0" drill="0.8128" diameter="1.6764" shape="square"/>
<pad name="A" x="-1.27" y="0" drill="0.8128" diameter="1.6764" shape="octagon"/>
<text x="6.35" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="6.35" y="-1.27" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="KA-3528ASYC" urn="urn:adsk.eagle:footprint:15682/1">
<description>&lt;b&gt;SURFACE MOUNT LED LAMP&lt;/b&gt; 3.5x2.8mm&lt;p&gt;
Source: http://www.kingbright.com/manager/upload/pdf/KA-3528ASYC(Ver1189474662.1)</description>
<wire x1="-1.55" y1="1.35" x2="1.55" y2="1.35" width="0.1016" layer="21"/>
<wire x1="1.55" y1="1.35" x2="1.55" y2="-1.35" width="0.1016" layer="51"/>
<wire x1="1.55" y1="-1.35" x2="-1.55" y2="-1.35" width="0.1016" layer="21"/>
<wire x1="-1.55" y1="-1.35" x2="-1.55" y2="1.35" width="0.1016" layer="51"/>
<wire x1="-0.65" y1="0.95" x2="0.65" y2="0.95" width="0.1016" layer="21" curve="-68.40813"/>
<wire x1="0.65" y1="-0.95" x2="-0.65" y2="-0.95" width="0.1016" layer="21" curve="-68.40813"/>
<circle x="0" y="0" radius="1.15" width="0.1016" layer="51"/>
<smd name="A" x="-1.55" y="0" dx="1.5" dy="2.2" layer="1"/>
<smd name="C" x="1.55" y="0" dx="1.5" dy="2.2" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.75" y1="0.6" x2="-1.6" y2="1.1" layer="51"/>
<rectangle x1="-1.75" y1="-1.1" x2="-1.6" y2="-0.6" layer="51"/>
<rectangle x1="1.6" y1="-1.1" x2="1.75" y2="-0.6" layer="51" rot="R180"/>
<rectangle x1="1.6" y1="0.6" x2="1.75" y2="1.1" layer="51" rot="R180"/>
<polygon width="0.1016" layer="51">
<vertex x="1.55" y="-1.35"/>
<vertex x="1.55" y="-0.625"/>
<vertex x="0.825" y="-1.35"/>
</polygon>
<polygon width="0.1016" layer="21">
<vertex x="1.55" y="-1.35"/>
<vertex x="1.55" y="-1.175"/>
<vertex x="1" y="-1.175"/>
<vertex x="0.825" y="-1.35"/>
</polygon>
</package>
<package name="SML0805" urn="urn:adsk.eagle:footprint:15683/1">
<description>&lt;b&gt;SML0805-2CW-TR (0805 PROFILE)&lt;/b&gt; COOL WHITE&lt;p&gt;
Source: http://www.ledtronics.com/ds/smd-0603/Dstr0093.pdf</description>
<wire x1="-0.95" y1="-0.55" x2="0.95" y2="-0.55" width="0.1016" layer="51"/>
<wire x1="0.95" y1="-0.55" x2="0.95" y2="0.55" width="0.1016" layer="51"/>
<wire x1="0.95" y1="0.55" x2="-0.95" y2="0.55" width="0.1016" layer="51"/>
<wire x1="-0.95" y1="0.55" x2="-0.95" y2="-0.55" width="0.1016" layer="51"/>
<wire x1="-0.175" y1="-0.025" x2="0" y2="0.15" width="0.0634" layer="21"/>
<wire x1="0" y1="0.15" x2="0.15" y2="0" width="0.0634" layer="21"/>
<wire x1="0.15" y1="0" x2="-0.025" y2="-0.175" width="0.0634" layer="21"/>
<wire x1="-0.025" y1="-0.175" x2="-0.175" y2="-0.025" width="0.0634" layer="21"/>
<circle x="-0.275" y="0.4" radius="0.125" width="0" layer="21"/>
<smd name="C" x="-1.05" y="0" dx="1.2" dy="1.2" layer="1"/>
<smd name="A" x="1.05" y="0" dx="1.2" dy="1.2" layer="1"/>
<text x="-1.5" y="1" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.5" y="-2" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="SML1206" urn="urn:adsk.eagle:footprint:15684/1">
<description>&lt;b&gt;SML10XXKH-TR (HIGH INTENSITY) LED&lt;/b&gt;&lt;p&gt;
&lt;table&gt;
&lt;tr&gt;&lt;td&gt;SML10R3KH-TR&lt;/td&gt;&lt;td&gt;ULTRA RED&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;SML10E3KH-TR&lt;/td&gt;&lt;td&gt;SUPER REDSUPER BLUE&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;SML10O3KH-TR&lt;/td&gt;&lt;td&gt;SUPER ORANGE&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;SML10PY3KH-TR&lt;/td&gt;&lt;td&gt;PURE YELLOW&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;SML10OY3KH-TR&lt;/td&gt;&lt;td&gt;ULTRA YELLOW&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;SML10AG3KH-TR&lt;/td&gt;&lt;td&gt;AQUA GREEN&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;SML10BG3KH-TR&lt;/td&gt;&lt;td&gt;BLUE GREEN&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;SML10PB1KH-TR&lt;/td&gt;&lt;td&gt;SUPER BLUE&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;SML10CW1KH-TR&lt;/td&gt;&lt;td&gt;WHITE&lt;/td&gt;&lt;/tr&gt;
&lt;/table&gt;

Source: http://www.ledtronics.com/ds/smd-1206/dstr0094.PDF</description>
<wire x1="-1.5" y1="0.5" x2="-1.5" y2="-0.5" width="0.2032" layer="51" curve="-180"/>
<wire x1="1.5" y1="-0.5" x2="1.5" y2="0.5" width="0.2032" layer="51" curve="-180"/>
<wire x1="-1.55" y1="0.75" x2="1.55" y2="0.75" width="0.1016" layer="51"/>
<wire x1="1.55" y1="-0.75" x2="-1.55" y2="-0.75" width="0.1016" layer="51"/>
<circle x="-0.725" y="0.525" radius="0.125" width="0" layer="21"/>
<smd name="C" x="-1.75" y="0" dx="1.5" dy="1.5" layer="1"/>
<smd name="A" x="1.75" y="0" dx="1.5" dy="1.5" layer="1"/>
<text x="-1.5" y="1" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.5" y="-2.5" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.6" y1="0.4" x2="-1.15" y2="0.8" layer="51"/>
<rectangle x1="-1.6" y1="-0.8" x2="-1.15" y2="-0.4" layer="51"/>
<rectangle x1="-1.175" y1="-0.6" x2="-1" y2="-0.275" layer="51"/>
<rectangle x1="1.15" y1="-0.8" x2="1.6" y2="-0.4" layer="51" rot="R180"/>
<rectangle x1="1.15" y1="0.4" x2="1.6" y2="0.8" layer="51" rot="R180"/>
<rectangle x1="1" y1="0.275" x2="1.175" y2="0.6" layer="51" rot="R180"/>
<rectangle x1="-0.1" y1="-0.1" x2="0.1" y2="0.1" layer="21"/>
</package>
<package name="SML0603" urn="urn:adsk.eagle:footprint:15685/1">
<description>&lt;b&gt;SML0603-XXX (HIGH INTENSITY) LED&lt;/b&gt;&lt;p&gt;
&lt;table&gt;
&lt;tr&gt;&lt;td&gt;AG3K&lt;/td&gt;&lt;td&gt;AQUA GREEN&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;B1K&lt;/td&gt;&lt;td&gt;SUPER BLUE&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;R1K&lt;/td&gt;&lt;td&gt;SUPER RED&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;R3K&lt;/td&gt;&lt;td&gt;ULTRA RED&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;O3K&lt;/td&gt;&lt;td&gt;SUPER ORANGE&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;O3KH&lt;/td&gt;&lt;td&gt;SOFT ORANGE&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;Y3KH&lt;/td&gt;&lt;td&gt;SUPER YELLOW&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;Y3K&lt;/td&gt;&lt;td&gt;SUPER YELLOW&lt;/td&gt;&lt;/tr&gt;
&lt;tr&gt;&lt;td&gt;2CW&lt;/td&gt;&lt;td&gt;WHITE&lt;/td&gt;&lt;/tr&gt;
&lt;/table&gt;
Source: http://www.ledtronics.com/ds/smd-0603/Dstr0092.pdf</description>
<wire x1="-0.75" y1="0.35" x2="0.75" y2="0.35" width="0.1016" layer="51"/>
<wire x1="0.75" y1="0.35" x2="0.75" y2="-0.35" width="0.1016" layer="51"/>
<wire x1="0.75" y1="-0.35" x2="-0.75" y2="-0.35" width="0.1016" layer="51"/>
<wire x1="-0.75" y1="-0.35" x2="-0.75" y2="0.35" width="0.1016" layer="51"/>
<wire x1="-0.45" y1="0.3" x2="-0.45" y2="-0.3" width="0.1016" layer="51"/>
<wire x1="0.45" y1="0.3" x2="0.45" y2="-0.3" width="0.1016" layer="51"/>
<wire x1="-0.2" y1="0.35" x2="0.2" y2="0.35" width="0.1016" layer="21"/>
<wire x1="-0.2" y1="-0.35" x2="0.2" y2="-0.35" width="0.1016" layer="21"/>
<smd name="C" x="-0.75" y="0" dx="0.8" dy="0.8" layer="1"/>
<smd name="A" x="0.75" y="0" dx="0.8" dy="0.8" layer="1"/>
<text x="-1" y="1" size="1.27" layer="25">&gt;NAME</text>
<text x="-1" y="-2" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.4" y1="0.175" x2="0" y2="0.4" layer="51"/>
<rectangle x1="-0.25" y1="0.175" x2="0" y2="0.4" layer="21"/>
</package>
</packages>
<packages3d>
<package3d name="LED5MM" urn="urn:adsk.eagle:package:15799/2" locally_modified="yes" type="model">
<description>LED
5 mm, round</description>
<packageinstances>
<packageinstance name="LED5MM"/>
</packageinstances>
</package3d>
<package3d name="1206" urn="urn:adsk.eagle:package:15796/2" type="model">
<description>CHICAGO MINIATURE LAMP, INC.
7022X Series SMT LEDs 1206 Package Size</description>
<packageinstances>
<packageinstance name="1206"/>
</packageinstances>
</package3d>
<package3d name="LD260" urn="urn:adsk.eagle:package:15794/1" type="box">
<description>LED
5 mm, square, Siemens</description>
<packageinstances>
<packageinstance name="LD260"/>
</packageinstances>
</package3d>
<package3d name="LED2X5" urn="urn:adsk.eagle:package:15800/1" type="box">
<description>LED
2 x 5 mm, rectangle</description>
<packageinstances>
<packageinstance name="LED2X5"/>
</packageinstances>
</package3d>
<package3d name="LED3MM" urn="urn:adsk.eagle:package:15797/1" type="box">
<description>LED
3 mm, round</description>
<packageinstances>
<packageinstance name="LED3MM"/>
</packageinstances>
</package3d>
<package3d name="LSU260" urn="urn:adsk.eagle:package:15805/1" type="box">
<description>LED
1 mm, round, Siemens</description>
<packageinstances>
<packageinstance name="LSU260"/>
</packageinstances>
</package3d>
<package3d name="LZR181" urn="urn:adsk.eagle:package:15808/1" type="box">
<description>LED BLOCK
1 LED, Siemens</description>
<packageinstances>
<packageinstance name="LZR181"/>
</packageinstances>
</package3d>
<package3d name="Q62902-B152" urn="urn:adsk.eagle:package:15803/1" type="box">
<description>LED HOLDER
Siemens</description>
<packageinstances>
<packageinstance name="Q62902-B152"/>
</packageinstances>
</package3d>
<package3d name="Q62902-B153" urn="urn:adsk.eagle:package:15804/1" type="box">
<description>LED HOLDER
Siemens</description>
<packageinstances>
<packageinstance name="Q62902-B153"/>
</packageinstances>
</package3d>
<package3d name="Q62902-B155" urn="urn:adsk.eagle:package:15807/1" type="box">
<description>LED HOLDER
Siemens</description>
<packageinstances>
<packageinstance name="Q62902-B155"/>
</packageinstances>
</package3d>
<package3d name="Q62902-B156" urn="urn:adsk.eagle:package:15806/1" type="box">
<description>LED HOLDER
Siemens</description>
<packageinstances>
<packageinstance name="Q62902-B156"/>
</packageinstances>
</package3d>
<package3d name="SFH480" urn="urn:adsk.eagle:package:15809/1" type="box">
<description>IR LED
infrared emitting diode, Infineon
TO-18, lead spacing 2.54 mm, cathode marking
Inifineon</description>
<packageinstances>
<packageinstance name="SFH480"/>
</packageinstances>
</package3d>
<package3d name="SFH482" urn="urn:adsk.eagle:package:15795/1" type="box">
<description>IR LED
infrared emitting diode, Infineon
TO-18, lead spacing 2.54 mm, cathode marking
Inifineon</description>
<packageinstances>
<packageinstance name="SFH482"/>
</packageinstances>
</package3d>
<package3d name="U57X32" urn="urn:adsk.eagle:package:15789/1" type="box">
<description>LED
rectangle, 5.7 x 3.2 mm</description>
<packageinstances>
<packageinstance name="U57X32"/>
</packageinstances>
</package3d>
<package3d name="IRL80A" urn="urn:adsk.eagle:package:15810/1" type="box">
<description>IR LED
IR transmitter Siemens</description>
<packageinstances>
<packageinstance name="IRL80A"/>
</packageinstances>
</package3d>
<package3d name="P-LCC-2" urn="urn:adsk.eagle:package:15817/1" type="box">
<description>TOPLED® High-optical Power LED (HOP)
Source: http://www.osram.convergy.de/ ... ls_t675.pdf</description>
<packageinstances>
<packageinstance name="P-LCC-2"/>
</packageinstances>
</package3d>
<package3d name="OSRAM-MINI-TOP-LED" urn="urn:adsk.eagle:package:15811/1" type="box">
<description>BLUE LINETM Hyper Mini TOPLED® Hyper-Bright LED
Source: http://www.osram.convergy.de/ ... LB M676.pdf</description>
<packageinstances>
<packageinstance name="OSRAM-MINI-TOP-LED"/>
</packageinstances>
</package3d>
<package3d name="OSRAM-SIDELED" urn="urn:adsk.eagle:package:15812/1" type="box">
<description>Super SIDELED® High-Current LED
LG A672, LP A672 
Source: http://www.osram.convergy.de/ ... LG_LP_A672.pdf (2004.05.13)</description>
<packageinstances>
<packageinstance name="OSRAM-SIDELED"/>
</packageinstances>
</package3d>
<package3d name="SMART-LED" urn="urn:adsk.eagle:package:15814/1" type="box">
<description>SmartLEDTM Hyper-Bright LED
Source: http://www.osram.convergy.de/ ... LA_LO_LS_LY L896.pdf</description>
<packageinstances>
<packageinstance name="SMART-LED"/>
</packageinstances>
</package3d>
<package3d name="P-LCC-2-TOPLED-RG" urn="urn:adsk.eagle:package:15813/1" type="box">
<description>Hyper TOPLED® RG Hyper-Bright LED
Source: http://www.osram.convergy.de/ ... LA_LO_LS_LY T776.pdf</description>
<packageinstances>
<packageinstance name="P-LCC-2-TOPLED-RG"/>
</packageinstances>
</package3d>
<package3d name="MICRO-SIDELED" urn="urn:adsk.eagle:package:15815/1" type="box">
<description>Hyper Micro SIDELED®
Source: http://www.osram.convergy.de/ ... LA_LO_LS_LY Y876.pdf</description>
<packageinstances>
<packageinstance name="MICRO-SIDELED"/>
</packageinstances>
</package3d>
<package3d name="P-LCC-4" urn="urn:adsk.eagle:package:15816/1" type="box">
<description>Power TOPLED®
Source: http://www.osram.convergy.de/ ... LA_LO_LA_LY E67B.pdf</description>
<packageinstances>
<packageinstance name="P-LCC-4"/>
</packageinstances>
</package3d>
<package3d name="CHIP-LED0603" urn="urn:adsk.eagle:package:15819/3" type="model">
<description>Hyper CHIPLED Hyper-Bright LED
LB Q993
Source: http://www.osram.convergy.de/ ... Lb_q993.pdf</description>
<packageinstances>
<packageinstance name="CHIP-LED0603"/>
</packageinstances>
</package3d>
<package3d name="CHIP-LED0805" urn="urn:adsk.eagle:package:15818/2" type="model">
<description>Hyper CHIPLED Hyper-Bright LED
LB R99A
Source: http://www.osram.convergy.de/ ... lb_r99a.pdf</description>
<packageinstances>
<packageinstance name="CHIP-LED0805"/>
</packageinstances>
</package3d>
<package3d name="MINI-TOPLED-SANTANA" urn="urn:adsk.eagle:package:15820/1" type="box">
<description>Mini TOPLED Santana®
Source: http://www.osram.convergy.de/ ... LG M470.pdf</description>
<packageinstances>
<packageinstance name="MINI-TOPLED-SANTANA"/>
</packageinstances>
</package3d>
<package3d name="CHIPLED_0805" urn="urn:adsk.eagle:package:15821/2" type="model">
<description>CHIPLED
Source: http://www.osram.convergy.de/ ... LG_R971.pdf</description>
<packageinstances>
<packageinstance name="CHIPLED_0805"/>
</packageinstances>
</package3d>
<package3d name="CHIPLED_1206" urn="urn:adsk.eagle:package:15823/2" type="model">
<description>CHIPLED
Source: http://www.osram.convergy.de/ ... LG_LY N971.pdf</description>
<packageinstances>
<packageinstance name="CHIPLED_1206"/>
</packageinstances>
</package3d>
<package3d name="CHIPLED_0603" urn="urn:adsk.eagle:package:15822/2" type="model">
<description>CHIPLED
Source: http://www.osram.convergy.de/ ... LG_LY Q971.pdf</description>
<packageinstances>
<packageinstance name="CHIPLED_0603"/>
</packageinstances>
</package3d>
<package3d name="CHIPLED-0603-TTW" urn="urn:adsk.eagle:package:15824/1" type="box">
<description>CHIPLED-0603
Recommended Solder Pad useable for SmartLEDTM and Chipled - Package 0603
Package able to withstand TTW-soldering heat
Package suitable for TTW-soldering
Source: http://www.osram.convergy.de/ ... LO_LS_LY L89K.pdf</description>
<packageinstances>
<packageinstance name="CHIPLED-0603-TTW"/>
</packageinstances>
</package3d>
<package3d name="SMARTLED-TTW" urn="urn:adsk.eagle:package:15825/1" type="box">
<description>SmartLED TTW
Recommended Solder Pad useable for SmartLEDTM and Chipled - Package 0603
Package able to withstand TTW-soldering heat
Package suitable for TTW-soldering
Source: http://www.osram.convergy.de/ ... LO_LS_LY L89K.pdf</description>
<packageinstances>
<packageinstance name="SMARTLED-TTW"/>
</packageinstances>
</package3d>
<package3d name="LUMILED+" urn="urn:adsk.eagle:package:15826/1" type="box">
<description>Lumileds Lighting. LUXEON® with cool pad
Source: K2.pdf</description>
<packageinstances>
<packageinstance name="LUMILED+"/>
</packageinstances>
</package3d>
<package3d name="LUMILED" urn="urn:adsk.eagle:package:15827/1" type="box">
<description>Lumileds Lighting. LUXEON® without cool pad
Source: K2.pdf</description>
<packageinstances>
<packageinstance name="LUMILED"/>
</packageinstances>
</package3d>
<package3d name="LED10MM" urn="urn:adsk.eagle:package:15828/1" type="box">
<description>LED
10 mm, round</description>
<packageinstances>
<packageinstance name="LED10MM"/>
</packageinstances>
</package3d>
<package3d name="KA-3528ASYC" urn="urn:adsk.eagle:package:15831/1" type="box">
<description>SURFACE MOUNT LED LAMP 3.5x2.8mm
Source: http://www.kingbright.com/manager/upload/pdf/KA-3528ASYC(Ver1189474662.1)</description>
<packageinstances>
<packageinstance name="KA-3528ASYC"/>
</packageinstances>
</package3d>
<package3d name="SML0805" urn="urn:adsk.eagle:package:15830/1" type="box">
<description>SML0805-2CW-TR (0805 PROFILE) COOL WHITE
Source: http://www.ledtronics.com/ds/smd-0603/Dstr0093.pdf</description>
<packageinstances>
<packageinstance name="SML0805"/>
</packageinstances>
</package3d>
<package3d name="SML1206" urn="urn:adsk.eagle:package:15829/1" type="box">
<description>SML10XXKH-TR (HIGH INTENSITY) LED

SML10R3KH-TRULTRA RED
SML10E3KH-TRSUPER REDSUPER BLUE
SML10O3KH-TRSUPER ORANGE
SML10PY3KH-TRPURE YELLOW
SML10OY3KH-TRULTRA YELLOW
SML10AG3KH-TRAQUA GREEN
SML10BG3KH-TRBLUE GREEN
SML10PB1KH-TRSUPER BLUE
SML10CW1KH-TRWHITE


Source: http://www.ledtronics.com/ds/smd-1206/dstr0094.PDF</description>
<packageinstances>
<packageinstance name="SML1206"/>
</packageinstances>
</package3d>
<package3d name="SML0603" urn="urn:adsk.eagle:package:15832/1" type="box">
<description>SML0603-XXX (HIGH INTENSITY) LED

AG3KAQUA GREEN
B1KSUPER BLUE
R1KSUPER RED
R3KULTRA RED
O3KSUPER ORANGE
O3KHSOFT ORANGE
Y3KHSUPER YELLOW
Y3KSUPER YELLOW
2CWWHITE

Source: http://www.ledtronics.com/ds/smd-0603/Dstr0092.pdf</description>
<packageinstances>
<packageinstance name="SML0603"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="LED" urn="urn:adsk.eagle:symbol:15639/2">
<wire x1="1.27" y1="0" x2="0" y2="-2.54" width="0.254" layer="94"/>
<wire x1="0" y1="-2.54" x2="-1.27" y2="0" width="0.254" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="0" y2="-2.54" width="0.254" layer="94"/>
<wire x1="0" y1="-2.54" x2="-1.27" y2="-2.54" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="0" width="0.254" layer="94"/>
<wire x1="-2.032" y1="-0.762" x2="-3.429" y2="-2.159" width="0.1524" layer="94"/>
<wire x1="-1.905" y1="-1.905" x2="-3.302" y2="-3.302" width="0.1524" layer="94"/>
<text x="3.556" y="-4.572" size="1.778" layer="95" rot="R90">&gt;NAME</text>
<text x="5.715" y="-4.572" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="C" x="0" y="-5.08" visible="off" length="short" direction="pas" rot="R90"/>
<pin name="A" x="0" y="2.54" visible="off" length="short" direction="pas" rot="R270"/>
<polygon width="0.1524" layer="94">
<vertex x="-3.429" y="-2.159"/>
<vertex x="-3.048" y="-1.27"/>
<vertex x="-2.54" y="-1.778"/>
</polygon>
<polygon width="0.1524" layer="94">
<vertex x="-3.302" y="-3.302"/>
<vertex x="-2.921" y="-2.413"/>
<vertex x="-2.413" y="-2.921"/>
</polygon>
</symbol>
</symbols>
<devicesets>
<deviceset name="LED" urn="urn:adsk.eagle:component:15916/9" locally_modified="yes" prefix="LED" uservalue="yes">
<description>&lt;b&gt;LED&lt;/b&gt;&lt;p&gt;
&lt;u&gt;OSRAM&lt;/u&gt;:&lt;br&gt;

- &lt;u&gt;CHIPLED&lt;/u&gt;&lt;br&gt;
LG R971, LG N971, LY N971, LG Q971, LY Q971, LO R971, LY R971
LH N974, LH R974&lt;br&gt;
LS Q976, LO Q976, LY Q976&lt;br&gt;
LO Q996&lt;br&gt;

- &lt;u&gt;Hyper CHIPLED&lt;/u&gt;&lt;br&gt;
LW Q18S&lt;br&gt;
LB Q993, LB Q99A, LB R99A&lt;br&gt;

- &lt;u&gt;SideLED&lt;/u&gt;&lt;br&gt;
LS A670, LO A670, LY A670, LG A670, LP A670&lt;br&gt;
LB A673, LV A673, LT A673, LW A673&lt;br&gt;
LH A674&lt;br&gt;
LY A675&lt;br&gt;
LS A676, LA A676, LO A676, LY A676, LW A676&lt;br&gt;
LS A679, LY A679, LG A679&lt;br&gt;

-  &lt;u&gt;Hyper Micro SIDELED®&lt;/u&gt;&lt;br&gt;
LS Y876, LA Y876, LO Y876, LY Y876&lt;br&gt;
LT Y87S&lt;br&gt;

- &lt;u&gt;SmartLED&lt;/u&gt;&lt;br&gt;
LW L88C, LW L88S&lt;br&gt;
LB L89C, LB L89S, LG L890&lt;br&gt;
LS L89K, LO L89K, LY L89K&lt;br&gt;
LS L896, LA L896, LO L896, LY L896&lt;br&gt;

- &lt;u&gt;TOPLED&lt;/u&gt;&lt;br&gt;
LS T670, LO T670, LY T670, LG T670, LP T670&lt;br&gt;
LSG T670, LSP T670, LSY T670, LOP T670, LYG T670&lt;br&gt;
LG T671, LOG T671, LSG T671&lt;br&gt;
LB T673, LV T673, LT T673, LW T673&lt;br&gt;
LH T674&lt;br&gt;
LS T676, LA T676, LO T676, LY T676, LB T676, LH T676, LSB T676, LW T676&lt;br&gt;
LB T67C, LV T67C, LT T67C, LS T67K, LO T67K, LY T67K, LW E67C&lt;br&gt;
LS E67B, LA E67B, LO E67B, LY E67B, LB E67C, LV E67C, LT E67C&lt;br&gt;
LW T67C&lt;br&gt;
LS T679, LY T679, LG T679&lt;br&gt;
LS T770, LO T770, LY T770, LG T770, LP T770&lt;br&gt;
LB T773, LV T773, LT T773, LW T773&lt;br&gt;
LH T774&lt;br&gt;
LS E675, LA E675, LY E675, LS T675&lt;br&gt;
LS T776, LA T776, LO T776, LY T776, LB T776&lt;br&gt;
LHGB T686&lt;br&gt;
LT T68C, LB T68C&lt;br&gt;

- &lt;u&gt;Hyper Mini TOPLED®&lt;/u&gt;&lt;br&gt;
LB M676&lt;br&gt;

- &lt;u&gt;Mini TOPLED Santana®&lt;/u&gt;&lt;br&gt;
LG M470&lt;br&gt;
LS M47K, LO M47K, LY M47K
&lt;p&gt;
Source: http://www.osram.convergy.de&lt;p&gt;

&lt;u&gt;LUXEON:&lt;/u&gt;&lt;br&gt;
- &lt;u&gt;LUMILED®&lt;/u&gt;&lt;br&gt;
LXK2-PW12-R00, LXK2-PW12-S00, LXK2-PW14-U00, LXK2-PW14-V00&lt;br&gt;
LXK2-PM12-R00, LXK2-PM12-S00, LXK2-PM14-U00&lt;br&gt;
LXK2-PE12-Q00, LXK2-PE12-R00, LXK2-PE12-S00, LXK2-PE14-T00, LXK2-PE14-U00&lt;br&gt;
LXK2-PB12-K00, LXK2-PB12-L00, LXK2-PB12-M00, LXK2-PB14-N00, LXK2-PB14-P00, LXK2-PB14-Q00&lt;br&gt;
LXK2-PR12-L00, LXK2-PR12-M00, LXK2-PR14-Q00, LXK2-PR14-R00&lt;br&gt;
LXK2-PD12-Q00, LXK2-PD12-R00, LXK2-PD12-S00&lt;br&gt;
LXK2-PH12-R00, LXK2-PH12-S00&lt;br&gt;
LXK2-PL12-P00, LXK2-PL12-Q00, LXK2-PL12-R00
&lt;p&gt;
Source: www.luxeon.com&lt;p&gt;

&lt;u&gt;KINGBRIGHT:&lt;/U&gt;&lt;p&gt;
KA-3528ASYC&lt;br&gt;
Source: www.kingbright.com</description>
<gates>
<gate name="G$1" symbol="LED" x="0" y="0"/>
</gates>
<devices>
<device name="SMT1206" package="1206">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15796/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="LD260" package="LD260">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15794/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SQR2X5" package="LED2X5">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15800/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="3MM" package="LED3MM">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15797/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="5MM" package="LED5MM">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15799/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="LSU260" package="LSU260">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15805/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="LZR181" package="LZR181">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15808/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="B152" package="Q62902-B152">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15803/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="B153" package="Q62902-B153">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15804/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="B155" package="Q62902-B155">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15807/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="B156" package="Q62902-B156">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15806/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SFH480" package="SFH480">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15809/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SFH482" package="SFH482">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15795/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SQR5.7X3.2" package="U57X32">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15789/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="IRL80A" package="IRL80A">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15810/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="P-LCC-2" package="P-LCC-2">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15817/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MINI-TOP" package="OSRAM-MINI-TOP-LED">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15811/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SIDELED" package="OSRAM-SIDELED">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15812/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SMART-LED" package="SMART-LED">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="B"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15814/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="P-LCC-2-BACK" package="P-LCC-2-TOPLED-RG">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15813/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="MICRO-SIDELED" package="MICRO-SIDELED">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15815/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="P-LCC-4" package="P-LCC-4">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C@4"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15816/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="CHIP-LED0603" package="CHIP-LED0603">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15819/3"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="CHIP-LED0805" package="CHIP-LED0805">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15818/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TOPLED-SANTANA" package="MINI-TOPLED-SANTANA">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15820/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="CHIPLED_0805" package="CHIPLED_0805">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15821/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="CHIPLED_1206" package="CHIPLED_1206">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15823/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="CHIPLED_0603" package="CHIPLED_0603">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15822/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="CHIPLED-0603-TTW" package="CHIPLED-0603-TTW">
<connects>
<connect gate="G$1" pin="A" pad="A@1"/>
<connect gate="G$1" pin="C" pad="C@1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15824/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="" package="SMARTLED-TTW">
<connects>
<connect gate="G$1" pin="A" pad="A@1"/>
<connect gate="G$1" pin="C" pad="C@1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15825/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="-LUMILED+" package="LUMILED+">
<connects>
<connect gate="G$1" pin="A" pad="2+"/>
<connect gate="G$1" pin="C" pad="4-"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15826/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="-LUMILED" package="LUMILED">
<connects>
<connect gate="G$1" pin="A" pad="2+"/>
<connect gate="G$1" pin="C" pad="4-"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15827/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="10MM" package="LED10MM">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="K"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15828/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="KA-3528ASYC" package="KA-3528ASYC">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15831/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SML0805" package="SML0805">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15830/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SML1206" package="SML1206">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15829/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SML0603" package="SML0603">
<connects>
<connect gate="G$1" pin="A" pad="A"/>
<connect gate="G$1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15832/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="diode_mod">
<description>&lt;b&gt;Diodes&lt;/b&gt;&lt;p&gt;
Based on the following sources:
&lt;ul&gt;
&lt;li&gt;Motorola : www.onsemi.com
&lt;li&gt;Fairchild : www.fairchildsemi.com
&lt;li&gt;Philips : www.semiconductors.com
&lt;li&gt;Vishay : www.vishay.de
&lt;/ul&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="DO41-10" urn="urn:adsk.eagle:footprint:43094/1" locally_modified="yes">
<description>&lt;B&gt;DIODE&lt;/B&gt;&lt;p&gt;
diameter 2.54 mm, horizontal, grid 10.16 mm</description>
<wire x1="2.032" y1="-1.27" x2="-2.032" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.032" y1="-1.27" x2="2.032" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-2.032" y1="1.27" x2="2.032" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-2.032" y1="1.27" x2="-2.032" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.08" y1="0" x2="4.064" y2="0" width="0.762" layer="51"/>
<wire x1="-5.08" y1="0" x2="-4.064" y2="0" width="0.762" layer="51"/>
<wire x1="-0.635" y1="0" x2="0" y2="0" width="0.1524" layer="21"/>
<wire x1="1.016" y1="0.635" x2="1.016" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="1.016" y1="-0.635" x2="0" y2="0" width="0.1524" layer="21"/>
<wire x1="0" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0" y1="0" x2="1.016" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="0.635" x2="0" y2="0" width="0.1524" layer="21"/>
<wire x1="0" y1="0" x2="0" y2="-0.635" width="0.1524" layer="21"/>
<pad name="A" x="5.08" y="0" drill="1.1176" diameter="2.1844"/>
<pad name="C" x="-5.08" y="0" drill="1.1176" diameter="2.1844"/>
<text x="-2.032" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.032" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-1.651" y1="-1.27" x2="-1.143" y2="1.27" layer="21"/>
<rectangle x1="2.032" y1="-0.381" x2="3.937" y2="0.381" layer="21"/>
<rectangle x1="-3.937" y1="-0.381" x2="-2.032" y2="0.381" layer="21"/>
</package>
</packages>
<packages3d>
<package3d name="DO41-10" urn="urn:adsk.eagle:package:43336/1" locally_modified="yes" type="box">
<description>DIODE
diameter 2.54 mm, horizontal, grid 10.16 mm</description>
<packageinstances>
<packageinstance name="DO41-10"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="D" urn="urn:adsk.eagle:symbol:43091/2">
<wire x1="-1.27" y1="-1.27" x2="1.27" y2="0" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="-1.27" y2="1.27" width="0.254" layer="94"/>
<wire x1="1.27" y1="1.27" x2="1.27" y2="0" width="0.254" layer="94"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0" width="0.254" layer="94"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="-1.27" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="1.27" y2="-1.27" width="0.254" layer="94"/>
<wire x1="-1.27" y1="0" x2="-2.54" y2="0" width="0.254" layer="94"/>
<wire x1="2.54" y1="0" x2="1.27" y2="0" width="0.254" layer="94"/>
<text x="2.54" y="0.4826" size="1.778" layer="95">&gt;NAME</text>
<text x="2.54" y="-2.3114" size="1.778" layer="96">&gt;VALUE</text>
<text x="-2.54" y="0" size="0.4064" layer="99" align="center">SpiceOrder 1</text>
<text x="2.54" y="0" size="0.4064" layer="99" align="center">SpiceOrder 2</text>
<pin name="A" x="-2.54" y="0" visible="off" length="point" direction="pas"/>
<pin name="C" x="2.54" y="0" visible="off" length="point" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="1N4004" urn="urn:adsk.eagle:component:43462/2" locally_modified="yes" prefix="D">
<description>&lt;B&gt;DIODE&lt;/B&gt;&lt;p&gt;
general purpose rectifier, 1 A</description>
<gates>
<gate name="1" symbol="D" x="0" y="0"/>
</gates>
<devices>
<device name="" package="DO41-10">
<connects>
<connect gate="1" pin="A" pad="A"/>
<connect gate="1" pin="C" pad="C"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:43336/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="eagle-ltspice_mod">
<description>Default symbols for import LTspice schematics&lt;p&gt;
2012-10-29 alf@cadsoft.de&lt;br&gt;</description>
<packages>
<package name="0207/10" urn="urn:adsk.eagle:footprint:13216/1" locally_modified="yes">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0207, grid 10 mm</description>
<wire x1="5.08" y1="0" x2="4.064" y2="0" width="0.6096" layer="51"/>
<wire x1="-5.08" y1="0" x2="-4.064" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.175" y1="0.889" x2="-2.921" y2="1.143" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-2.921" y2="-1.143" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="-1.143" x2="3.175" y2="-0.889" width="0.1524" layer="21" curve="90"/>
<wire x1="2.921" y1="1.143" x2="3.175" y2="0.889" width="0.1524" layer="21" curve="-90"/>
<wire x1="-3.175" y1="-0.889" x2="-3.175" y2="0.889" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="1.143" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="-2.921" y1="-1.143" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="-1.016" x2="-2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="-2.413" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.921" y1="1.143" x2="2.54" y2="1.143" width="0.1524" layer="21"/>
<wire x1="2.921" y1="-1.143" x2="2.54" y2="-1.143" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-0.889" x2="3.175" y2="0.889" width="0.1524" layer="21"/>
<rectangle x1="3.175" y1="-0.3048" x2="4.0386" y2="0.3048" layer="21"/>
<rectangle x1="-4.0386" y1="-0.3048" x2="-3.175" y2="0.3048" layer="21"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" diameter="2.1844"/>
<pad name="2" x="5.08" y="0" drill="0.8128" diameter="2.1844"/>
<text x="-3.048" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.2606" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="0204/7" urn="urn:adsk.eagle:footprint:13215/1" locally_modified="yes">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
type 0204, grid 7.5 mm</description>
<wire x1="3.81" y1="0" x2="2.921" y2="0" width="0.508" layer="51"/>
<wire x1="-3.81" y1="0" x2="-2.921" y2="0" width="0.508" layer="51"/>
<wire x1="-2.54" y1="0.762" x2="-2.286" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="-0.762" x2="-2.286" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="-1.016" x2="2.54" y2="-0.762" width="0.1524" layer="21" curve="90"/>
<wire x1="2.286" y1="1.016" x2="2.54" y2="0.762" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.54" y1="-0.762" x2="-2.54" y2="0.762" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="1.016" x2="-1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-1.778" y1="0.889" x2="-1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="-1.016" x2="-1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-1.778" y1="-0.889" x2="-1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.889" x2="1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="0.889" x2="-1.778" y2="0.889" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.889" x2="1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="1.778" y1="-0.889" x2="-1.778" y2="-0.889" width="0.1524" layer="21"/>
<wire x1="2.286" y1="1.016" x2="1.905" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.286" y1="-1.016" x2="1.905" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-0.762" x2="2.54" y2="0.762" width="0.1524" layer="21"/>
<rectangle x1="2.54" y1="-0.254" x2="2.921" y2="0.254" layer="21"/>
<rectangle x1="-2.921" y1="-0.254" x2="-2.54" y2="0.254" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" diameter="2.1844"/>
<pad name="2" x="3.81" y="0" drill="0.8128" diameter="2.1844"/>
<text x="-2.54" y="1.2954" size="0.9906" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.6256" y="-0.4826" size="0.9906" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="R0201" urn="urn:adsk.eagle:footprint:13233/1">
<description>&lt;b&gt;RESISTOR&lt;/b&gt; chip&lt;p&gt;
Source: http://www.vishay.com/docs/20008/dcrcw.pdf</description>
<rectangle x1="-0.3" y1="-0.15" x2="-0.15" y2="0.15" layer="51"/>
<rectangle x1="0.15" y1="-0.15" x2="0.3" y2="0.15" layer="51"/>
<rectangle x1="-0.15" y1="-0.15" x2="0.15" y2="0.15" layer="21"/>
<smd name="1" x="-0.255" y="0" dx="0.28" dy="0.43" layer="1"/>
<smd name="2" x="0.255" y="0" dx="0.28" dy="0.43" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R0402" urn="urn:adsk.eagle:footprint:13234/1">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.245" y1="0.224" x2="0.245" y2="0.224" width="0.1524" layer="51"/>
<wire x1="0.245" y1="-0.224" x2="-0.245" y2="-0.224" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.483" x2="1.473" y2="0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.483" x2="1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.483" x2="-1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.483" x2="-1.473" y2="0.483" width="0.0508" layer="39"/>
<rectangle x1="-0.554" y1="-0.3048" x2="-0.254" y2="0.2951" layer="51"/>
<rectangle x1="0.2588" y1="-0.3048" x2="0.5588" y2="0.2951" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
<smd name="1" x="-0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<smd name="2" x="0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R0603" urn="urn:adsk.eagle:footprint:13235/1">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.432" y1="-0.356" x2="0.432" y2="-0.356" width="0.1524" layer="51"/>
<wire x1="0.432" y1="0.356" x2="-0.432" y2="0.356" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.983" x2="1.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.983" x2="1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.983" x2="-1.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.983" x2="-1.473" y2="0.983" width="0.0508" layer="39"/>
<rectangle x1="0.4318" y1="-0.4318" x2="0.8382" y2="0.4318" layer="51"/>
<rectangle x1="-0.8382" y1="-0.4318" x2="-0.4318" y2="0.4318" layer="51"/>
<rectangle x1="-0.1999" y1="-0.4001" x2="0.1999" y2="0.4001" layer="35"/>
<smd name="1" x="-0.85" y="0" dx="1" dy="1.1" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1" dy="1.1" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R0805" urn="urn:adsk.eagle:footprint:13236/1">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1999" y1="-0.5001" x2="0.1999" y2="0.5001" layer="35"/>
<smd name="1" x="-0.95" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.95" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R1005" urn="urn:adsk.eagle:footprint:13237/1">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.245" y1="0.224" x2="0.245" y2="0.224" width="0.1524" layer="51"/>
<wire x1="0.245" y1="-0.224" x2="-0.245" y2="-0.224" width="0.1524" layer="51"/>
<wire x1="-1.473" y1="0.483" x2="1.473" y2="0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="0.483" x2="1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="1.473" y1="-0.483" x2="-1.473" y2="-0.483" width="0.0508" layer="39"/>
<wire x1="-1.473" y1="-0.483" x2="-1.473" y2="0.483" width="0.0508" layer="39"/>
<rectangle x1="-0.554" y1="-0.3048" x2="-0.254" y2="0.2951" layer="51"/>
<rectangle x1="0.2588" y1="-0.3048" x2="0.5588" y2="0.2951" layer="51"/>
<rectangle x1="-0.1999" y1="-0.3" x2="0.1999" y2="0.3" layer="35"/>
<smd name="1" x="-0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<smd name="2" x="0.65" y="0" dx="0.7" dy="0.9" layer="1"/>
<text x="-0.635" y="0.635" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-1.905" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R1206" urn="urn:adsk.eagle:footprint:13238/1">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="0.9525" y1="-0.8128" x2="-0.9652" y2="-0.8128" width="0.1524" layer="51"/>
<wire x1="0.9525" y1="0.8128" x2="-0.9652" y2="0.8128" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<rectangle x1="-1.6891" y1="-0.8763" x2="-0.9525" y2="0.8763" layer="51"/>
<rectangle x1="0.9525" y1="-0.8763" x2="1.6891" y2="0.8763" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
<smd name="1" x="-1.422" y="0" dx="1.6" dy="1.803" layer="1"/>
<smd name="2" x="1.422" y="0" dx="1.6" dy="1.803" layer="1"/>
<text x="-1.27" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.27" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R1210" urn="urn:adsk.eagle:footprint:13239/1">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-0.8999" x2="0.3" y2="0.8999" layer="35"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R1218" urn="urn:adsk.eagle:footprint:13240/1">
<description>&lt;b&gt;CRCW1218 Thick Film, Rectangular Chip Resistors&lt;/b&gt;&lt;p&gt;
Source: http://www.vishay.com .. dcrcw.pdf</description>
<wire x1="-0.913" y1="-2.219" x2="0.939" y2="-2.219" width="0.1524" layer="51"/>
<wire x1="0.913" y1="2.219" x2="-0.939" y2="2.219" width="0.1524" layer="51"/>
<rectangle x1="-1.651" y1="-2.3" x2="-0.9009" y2="2.3" layer="51"/>
<rectangle x1="0.9144" y1="-2.3" x2="1.6645" y2="2.3" layer="51"/>
<smd name="1" x="-1.475" y="0" dx="1.05" dy="4.9" layer="1"/>
<smd name="2" x="1.475" y="0" dx="1.05" dy="4.9" layer="1"/>
<text x="-2.54" y="2.54" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.81" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R2010" urn="urn:adsk.eagle:footprint:13241/1">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
<smd name="1" x="-2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<smd name="2" x="2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<text x="-3.175" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R2012" urn="urn:adsk.eagle:footprint:13242/1">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.41" y1="0.635" x2="0.41" y2="0.635" width="0.1524" layer="51"/>
<wire x1="-0.41" y1="-0.635" x2="0.41" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="-1.973" y1="0.983" x2="1.973" y2="0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="0.983" x2="1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="1.973" y1="-0.983" x2="-1.973" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-1.973" y1="-0.983" x2="-1.973" y2="0.983" width="0.0508" layer="39"/>
<rectangle x1="0.4064" y1="-0.6985" x2="1.0564" y2="0.7015" layer="51"/>
<rectangle x1="-1.0668" y1="-0.6985" x2="-0.4168" y2="0.7015" layer="51"/>
<rectangle x1="-0.1001" y1="-0.5999" x2="0.1001" y2="0.5999" layer="35"/>
<smd name="1" x="-0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<smd name="2" x="0.85" y="0" dx="1.3" dy="1.5" layer="1"/>
<text x="-0.635" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-0.635" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R2512" urn="urn:adsk.eagle:footprint:13243/1">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
<smd name="1" x="-2.8" y="0" dx="1.8" dy="3.2" layer="1"/>
<smd name="2" x="2.8" y="0" dx="1.8" dy="3.2" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R3216" urn="urn:adsk.eagle:footprint:13244/1">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.913" y1="0.8" x2="0.888" y2="0.8" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-0.8" x2="0.888" y2="-0.8" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="0.983" x2="2.473" y2="0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="0.983" x2="2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-0.983" x2="-2.473" y2="-0.983" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-0.983" x2="-2.473" y2="0.983" width="0.0508" layer="39"/>
<rectangle x1="-1.651" y1="-0.8763" x2="-0.9009" y2="0.8738" layer="51"/>
<rectangle x1="0.889" y1="-0.8763" x2="1.6391" y2="0.8738" layer="51"/>
<rectangle x1="-0.3" y1="-0.7" x2="0.3" y2="0.7" layer="35"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="1.8" layer="1"/>
<text x="-1.905" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R3225" urn="urn:adsk.eagle:footprint:13245/1">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-0.913" y1="1.219" x2="0.939" y2="1.219" width="0.1524" layer="51"/>
<wire x1="-0.913" y1="-1.219" x2="0.939" y2="-1.219" width="0.1524" layer="51"/>
<wire x1="-2.473" y1="1.483" x2="2.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="1.483" x2="2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="2.473" y1="-1.483" x2="-2.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-2.473" y1="-1.483" x2="-2.473" y2="1.483" width="0.0508" layer="39"/>
<rectangle x1="-1.651" y1="-1.3081" x2="-0.9009" y2="1.2918" layer="51"/>
<rectangle x1="0.9144" y1="-1.3081" x2="1.6645" y2="1.2918" layer="51"/>
<rectangle x1="-0.3" y1="-1" x2="0.3" y2="1" layer="35"/>
<smd name="1" x="-1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<smd name="2" x="1.4" y="0" dx="1.6" dy="2.7" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R4527" urn="urn:adsk.eagle:footprint:13246/1">
<description>&lt;b&gt;Package 4527&lt;/b&gt;&lt;p&gt;
Source: http://www.vishay.com/docs/31059/wsrhigh.pdf</description>
<wire x1="-5.675" y1="-3.375" x2="5.65" y2="-3.375" width="0.2032" layer="21"/>
<wire x1="5.65" y1="-3.375" x2="5.65" y2="3.375" width="0.2032" layer="51"/>
<wire x1="5.65" y1="3.375" x2="-5.675" y2="3.375" width="0.2032" layer="21"/>
<wire x1="-5.675" y1="3.375" x2="-5.675" y2="-3.375" width="0.2032" layer="51"/>
<smd name="1" x="-4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<smd name="2" x="4.575" y="0" dx="3.94" dy="5.84" layer="1"/>
<text x="-5.715" y="3.81" size="1.27" layer="25">&gt;NAME</text>
<text x="-5.715" y="-5.08" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R5025" urn="urn:adsk.eagle:footprint:13247/1">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;</description>
<wire x1="-1.662" y1="1.245" x2="1.662" y2="1.245" width="0.1524" layer="51"/>
<wire x1="-1.637" y1="-1.245" x2="1.687" y2="-1.245" width="0.1524" layer="51"/>
<wire x1="-3.473" y1="1.483" x2="3.473" y2="1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="1.483" x2="3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="3.473" y1="-1.483" x2="-3.473" y2="-1.483" width="0.0508" layer="39"/>
<wire x1="-3.473" y1="-1.483" x2="-3.473" y2="1.483" width="0.0508" layer="39"/>
<rectangle x1="-2.4892" y1="-1.3208" x2="-1.6393" y2="1.3292" layer="51"/>
<rectangle x1="1.651" y1="-1.3208" x2="2.5009" y2="1.3292" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
<smd name="1" x="-2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<smd name="2" x="2.2" y="0" dx="1.8" dy="2.7" layer="1"/>
<text x="-3.175" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.175" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="R6332" urn="urn:adsk.eagle:footprint:13248/1">
<description>&lt;b&gt;RESISTOR&lt;/b&gt;&lt;p&gt;
Source: http://download.siliconexpert.com/pdfs/2005/02/24/Semi_Ap/2/VSH/Resistor/dcrcwfre.pdf</description>
<wire x1="-2.362" y1="1.473" x2="2.387" y2="1.473" width="0.1524" layer="51"/>
<wire x1="-2.362" y1="-1.473" x2="2.387" y2="-1.473" width="0.1524" layer="51"/>
<wire x1="-3.973" y1="1.983" x2="3.973" y2="1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="1.983" x2="3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="3.973" y1="-1.983" x2="-3.973" y2="-1.983" width="0.0508" layer="39"/>
<wire x1="-3.973" y1="-1.983" x2="-3.973" y2="1.983" width="0.0508" layer="39"/>
<rectangle x1="-3.2004" y1="-1.5494" x2="-2.3505" y2="1.5507" layer="51"/>
<rectangle x1="2.3622" y1="-1.5494" x2="3.2121" y2="1.5507" layer="51"/>
<rectangle x1="-0.5001" y1="-1" x2="0.5001" y2="1" layer="35"/>
<smd name="1" x="-3.1" y="0" dx="1" dy="3.2" layer="1"/>
<smd name="2" x="3.1" y="0" dx="1" dy="3.2" layer="1"/>
<text x="-2.54" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="0207/10" urn="urn:adsk.eagle:package:13275/1" locally_modified="yes" type="box">
<description>RESISTOR
type 0207, grid 10 mm</description>
<packageinstances>
<packageinstance name="0207/10"/>
</packageinstances>
</package3d>
<package3d name="0204/7" urn="urn:adsk.eagle:package:13274/1" locally_modified="yes" type="box">
<description>RESISTOR
type 0204, grid 7.5 mm</description>
<packageinstances>
<packageinstance name="0204/7"/>
</packageinstances>
</package3d>
<package3d name="R0201" urn="urn:adsk.eagle:package:13294/1" type="box">
<description>RESISTOR chip
Source: http://www.vishay.com/docs/20008/dcrcw.pdf</description>
<packageinstances>
<packageinstance name="R0201"/>
</packageinstances>
</package3d>
<package3d name="R0402" urn="urn:adsk.eagle:package:13296/1" type="box">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R0402"/>
</packageinstances>
</package3d>
<package3d name="R0603" urn="urn:adsk.eagle:package:13302/1" type="box">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R0603"/>
</packageinstances>
</package3d>
<package3d name="R0805" urn="urn:adsk.eagle:package:13300/1" type="box">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R0805"/>
</packageinstances>
</package3d>
<package3d name="R1005" urn="urn:adsk.eagle:package:13297/1" type="box">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R1005"/>
</packageinstances>
</package3d>
<package3d name="R1206" urn="urn:adsk.eagle:package:13301/1" type="box">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R1206"/>
</packageinstances>
</package3d>
<package3d name="R1210" urn="urn:adsk.eagle:package:13299/1" type="box">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R1210"/>
</packageinstances>
</package3d>
<package3d name="R1218" urn="urn:adsk.eagle:package:13303/1" type="box">
<description>CRCW1218 Thick Film, Rectangular Chip Resistors
Source: http://www.vishay.com .. dcrcw.pdf</description>
<packageinstances>
<packageinstance name="R1218"/>
</packageinstances>
</package3d>
<package3d name="R2010" urn="urn:adsk.eagle:package:13309/1" type="box">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R2010"/>
</packageinstances>
</package3d>
<package3d name="R2012" urn="urn:adsk.eagle:package:13306/1" type="box">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R2012"/>
</packageinstances>
</package3d>
<package3d name="R2512" urn="urn:adsk.eagle:package:13304/1" type="box">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R2512"/>
</packageinstances>
</package3d>
<package3d name="R3216" urn="urn:adsk.eagle:package:13305/1" type="box">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R3216"/>
</packageinstances>
</package3d>
<package3d name="R3225" urn="urn:adsk.eagle:package:13311/1" type="box">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R3225"/>
</packageinstances>
</package3d>
<package3d name="R4527" urn="urn:adsk.eagle:package:13310/1" type="box">
<description>Package 4527
Source: http://www.vishay.com/docs/31059/wsrhigh.pdf</description>
<packageinstances>
<packageinstance name="R4527"/>
</packageinstances>
</package3d>
<package3d name="R5025" urn="urn:adsk.eagle:package:13308/1" type="box">
<description>RESISTOR</description>
<packageinstances>
<packageinstance name="R5025"/>
</packageinstances>
</package3d>
<package3d name="R6332" urn="urn:adsk.eagle:package:13307/1" type="box">
<description>RESISTOR
Source: http://download.siliconexpert.com/pdfs/2005/02/24/Semi_Ap/2/VSH/Resistor/dcrcwfre.pdf</description>
<packageinstances>
<packageinstance name="R6332"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="R" urn="urn:adsk.eagle:symbol:13232/1">
<wire x1="-2.54" y1="-0.889" x2="2.54" y2="-0.889" width="0.254" layer="94"/>
<wire x1="2.54" y1="0.889" x2="-2.54" y2="0.889" width="0.254" layer="94"/>
<wire x1="2.54" y1="-0.889" x2="2.54" y2="0.889" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-0.889" x2="-2.54" y2="0.889" width="0.254" layer="94"/>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
<pin name="2" x="5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
<text x="-3.81" y="1.4986" size="1.778" layer="95">&gt;NAME</text>
<text x="-3.81" y="-3.302" size="1.778" layer="96">&gt;VALUE</text>
<text x="-5.08" y="0" size="0.4064" layer="99" align="center">SpiceOrder 1</text>
<text x="5.08" y="0" size="0.4064" layer="99" align="center">SpiceOrder 2</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="R" urn="urn:adsk.eagle:component:13322/1" locally_modified="yes" prefix="R" uservalue="yes">
<description>&lt;B&gt;RESISTOR&lt;/B&gt;, European symbol</description>
<gates>
<gate name="G$1" symbol="R" x="0" y="0"/>
</gates>
<devices>
<device name="0204/7" package="0204/7">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13274/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="0207/10" package="0207/10">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13275/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R0201" package="R0201">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13294/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R0402" package="R0402">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13296/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R0603" package="R0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13302/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="" package="R0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13300/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R1005" package="R1005">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13297/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R1206" package="R1206">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13301/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R1210" package="R1210">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13299/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R1218" package="R1218">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13303/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R2010" package="R2010">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13309/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R2012" package="R2012">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13306/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R2512" package="R2512">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13304/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R3216" package="R3216">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13305/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R3225" package="R3225">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13311/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R4527" package="R4527">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13310/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R5025" package="R5025">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13308/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
<device name="R6332" package="R6332">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:13307/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="SPICEMODEL" value="NONE" constant="no"/>
<attribute name="SPICEPREFIX" value="R" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="jumper" urn="urn:adsk.eagle:library:252">
<description>&lt;b&gt;Jumpers&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="05" urn="urn:adsk.eagle:footprint:15423/1" library_version="1">
<description>&lt;b&gt;JUMPER&lt;/b&gt;</description>
<wire x1="-1.3462" y1="0" x2="1.3462" y2="0" width="0.6096" layer="21"/>
<wire x1="2.54" y1="0" x2="1.397" y2="0" width="0.6096" layer="51"/>
<wire x1="-2.54" y1="0" x2="-1.397" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-2.54" y="0.889" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="2.5001" y="0" size="0.02" layer="27">&gt;VALUE</text>
</package>
<package name="07" urn="urn:adsk.eagle:footprint:15424/1" library_version="1">
<description>&lt;b&gt;JUMPER&lt;/b&gt;</description>
<wire x1="-2.6162" y1="0" x2="2.6162" y2="0" width="0.6096" layer="21"/>
<wire x1="3.81" y1="0" x2="2.54" y2="0" width="0.6096" layer="51"/>
<wire x1="-3.81" y1="0" x2="-2.54" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-2.54" y="0.635" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="3.7" y="0" size="0.02" layer="27">&gt;VALUE</text>
</package>
<package name="10" urn="urn:adsk.eagle:footprint:15425/1" library_version="1">
<description>&lt;b&gt;JUMPER&lt;/b&gt;</description>
<wire x1="-3.8862" y1="0" x2="3.8862" y2="0" width="0.6096" layer="21"/>
<wire x1="5.08" y1="0" x2="3.937" y2="0" width="0.6096" layer="51"/>
<wire x1="-5.08" y1="0" x2="-3.937" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-3.175" y="0.635" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="5" y="0" size="0.02" layer="27">&gt;VALUE</text>
</package>
<package name="12" urn="urn:adsk.eagle:footprint:15426/1" library_version="1">
<description>&lt;b&gt;JUMPER&lt;/b&gt;</description>
<wire x1="-5.1562" y1="0" x2="5.1562" y2="0" width="0.6096" layer="21"/>
<wire x1="6.35" y1="0" x2="5.207" y2="0" width="0.6096" layer="51"/>
<wire x1="-6.35" y1="0" x2="-5.207" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-6.35" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-3.175" y="0.635" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="6.3" y="0" size="0.02" layer="27">&gt;VALUE</text>
</package>
<package name="15" urn="urn:adsk.eagle:footprint:15427/1" library_version="1">
<description>&lt;b&gt;JUMPER&lt;/b&gt;</description>
<wire x1="-6.4262" y1="0" x2="6.4262" y2="0" width="0.6096" layer="21"/>
<wire x1="7.62" y1="0" x2="6.477" y2="0" width="0.6096" layer="51"/>
<wire x1="-7.62" y1="0" x2="-6.477" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-7.62" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="7.62" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-5.715" y="0.635" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="7.5999" y="0" size="0.02" layer="27">&gt;VALUE</text>
</package>
<package name="20" urn="urn:adsk.eagle:footprint:15428/1" library_version="1">
<description>&lt;b&gt;JUMPER&lt;/b&gt;</description>
<wire x1="-8.9662" y1="0" x2="8.9662" y2="0" width="0.6096" layer="21"/>
<wire x1="10.16" y1="0" x2="9.017" y2="0" width="0.6096" layer="51"/>
<wire x1="-10.16" y1="0" x2="-9.017" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-10.16" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="10.16" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-8.255" y="0.635" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="10.1001" y="0" size="0.02" layer="27">&gt;VALUE</text>
</package>
<package name="25" urn="urn:adsk.eagle:footprint:15429/1" library_version="1">
<description>&lt;b&gt;JUMPER&lt;/b&gt;</description>
<wire x1="-11.5062" y1="0" x2="11.5062" y2="0" width="0.6096" layer="21"/>
<wire x1="12.7" y1="0" x2="11.557" y2="0" width="0.6096" layer="51"/>
<wire x1="-12.7" y1="0" x2="-11.557" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-12.7" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="12.7" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-10.795" y="0.635" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="12.5999" y="0" size="0.02" layer="27">&gt;VALUE</text>
</package>
<package name="30" urn="urn:adsk.eagle:footprint:15430/1" library_version="1">
<description>&lt;b&gt;JUMPER&lt;/b&gt;</description>
<wire x1="-14.0462" y1="0" x2="14.0462" y2="0" width="0.6096" layer="21"/>
<wire x1="15.24" y1="0" x2="14.097" y2="0" width="0.6096" layer="51"/>
<wire x1="-15.24" y1="0" x2="-14.097" y2="0" width="0.6096" layer="51"/>
<pad name="1" x="-15.24" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="15.24" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-13.335" y="0.635" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="15.2001" y="0" size="0.02" layer="27">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="05" urn="urn:adsk.eagle:package:15465/1" type="box" library_version="1">
<description>JUMPER</description>
<packageinstances>
<packageinstance name="05"/>
</packageinstances>
</package3d>
<package3d name="07" urn="urn:adsk.eagle:package:15466/1" type="box" library_version="1">
<description>JUMPER</description>
<packageinstances>
<packageinstance name="07"/>
</packageinstances>
</package3d>
<package3d name="10" urn="urn:adsk.eagle:package:15468/1" type="box" library_version="1">
<description>JUMPER</description>
<packageinstances>
<packageinstance name="10"/>
</packageinstances>
</package3d>
<package3d name="12" urn="urn:adsk.eagle:package:15467/1" type="box" library_version="1">
<description>JUMPER</description>
<packageinstances>
<packageinstance name="12"/>
</packageinstances>
</package3d>
<package3d name="15" urn="urn:adsk.eagle:package:15475/1" type="box" library_version="1">
<description>JUMPER</description>
<packageinstances>
<packageinstance name="15"/>
</packageinstances>
</package3d>
<package3d name="20" urn="urn:adsk.eagle:package:15473/1" type="box" library_version="1">
<description>JUMPER</description>
<packageinstances>
<packageinstance name="20"/>
</packageinstances>
</package3d>
<package3d name="25" urn="urn:adsk.eagle:package:15470/1" type="box" library_version="1">
<description>JUMPER</description>
<packageinstances>
<packageinstance name="25"/>
</packageinstances>
</package3d>
<package3d name="30" urn="urn:adsk.eagle:package:15469/1" type="box" library_version="1">
<description>JUMPER</description>
<packageinstances>
<packageinstance name="30"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="J" urn="urn:adsk.eagle:symbol:15422/1" library_version="1">
<wire x1="-2.54" y1="0" x2="-1.27" y2="1.27" width="0.1524" layer="94"/>
<wire x1="-1.27" y1="1.27" x2="1.27" y2="1.27" width="0.1524" layer="94"/>
<wire x1="2.54" y1="0" x2="1.27" y2="1.27" width="0.1524" layer="94"/>
<circle x="-2.54" y="0" radius="0.254" width="0.4064" layer="94"/>
<circle x="2.54" y="0" radius="0.254" width="0.4064" layer="94"/>
<text x="-2.54" y="1.905" size="1.778" layer="95">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="-5.08" y="0" visible="pad" length="short" direction="pas" swaplevel="1"/>
<pin name="2" x="5.08" y="0" visible="pad" length="short" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="J" urn="urn:adsk.eagle:component:15504/1" prefix="J" library_version="1">
<description>Bridge</description>
<gates>
<gate name="1" symbol="J" x="0" y="0"/>
</gates>
<devices>
<device name="5MM" package="05">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15465/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="7MM" package="07">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15466/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="10MM" package="10">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15468/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="12MM" package="12">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15467/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="15MM" package="15">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15475/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="20MM" package="20">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15473/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="25MM" package="25">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15470/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="30MM" package="30">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:15469/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="con-stocko_mod">
<description>&lt;b&gt;STOCKO Connector&lt;/b&gt;&lt;p&gt;
http://www.stocko.de&lt;br&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="MKS1852-6-0-202" urn="urn:adsk.eagle:footprint:10030/1" locally_modified="yes">
<description>&lt;b&gt;Serie MKS 1850&lt;/b&gt; Lötstifte In-line, Raster 2.5 mm, Stiftwannen, stehend, mit Rastlasche&lt;p&gt;
Source: http://www.stocko.de/de/index.php?seite=ka_rfk2_mks1850.html&amp;rubrik=Katalog+-+RFK+2&lt;br&gt;
rfk_1850.pdf</description>
<wire x1="-2.7283" y1="-3.6954" x2="-3.4912" y2="-3.6954" width="0.2032" layer="21"/>
<wire x1="-3.4912" y1="-3.6954" x2="-4.731" y2="-2.4556" width="0.2032" layer="21" curve="-90"/>
<wire x1="-4.731" y1="-2.4556" x2="-4.731" y2="2.4558" width="0.2032" layer="21"/>
<wire x1="-4.731" y1="2.4558" x2="-3.4912" y2="3.6956" width="0.2032" layer="21" curve="-90"/>
<wire x1="-3.4912" y1="3.6956" x2="3.4672" y2="3.6956" width="0.2032" layer="21"/>
<wire x1="3.4672" y1="3.6956" x2="4.7547" y2="2.4081" width="0.2032" layer="21" curve="-90"/>
<wire x1="4.7547" y1="2.4081" x2="4.7547" y2="-2.4079" width="0.2032" layer="21"/>
<wire x1="4.7547" y1="-2.4079" x2="3.4672" y2="-3.6954" width="0.2032" layer="21" curve="-90"/>
<wire x1="3.4672" y1="-3.6954" x2="2.7281" y2="-3.6954" width="0.2032" layer="21"/>
<wire x1="2.7281" y1="-3.6954" x2="2.7281" y2="-2.1695" width="0.2032" layer="21"/>
<wire x1="2.7281" y1="-2.1695" x2="2.2036" y2="-1.645" width="0.2032" layer="21" curve="90"/>
<wire x1="2.2036" y1="-1.645" x2="-2.2038" y2="-1.645" width="0.2032" layer="21"/>
<wire x1="-2.2038" y1="-1.645" x2="-2.7283" y2="-2.1695" width="0.2032" layer="21" curve="90"/>
<wire x1="-2.7283" y1="-2.1695" x2="-2.7283" y2="-3.6954" width="0.2032" layer="21"/>
<wire x1="-2.7521" y1="-2.6702" x2="-3.4674" y2="-2.6702" width="0.2032" layer="21"/>
<wire x1="-3.4674" y1="-2.6702" x2="-3.825" y2="-2.3126" width="0.2032" layer="21" curve="-90"/>
<wire x1="-3.825" y1="-2.3126" x2="-3.825" y2="2.3605" width="0.2032" layer="21"/>
<wire x1="-3.825" y1="2.3605" x2="-3.5151" y2="2.6704" width="0.2032" layer="21" curve="-90"/>
<wire x1="-3.5151" y1="2.6704" x2="3.4911" y2="2.6704" width="0.2032" layer="21"/>
<wire x1="3.4911" y1="2.6704" x2="3.8249" y2="2.3366" width="0.2032" layer="21" curve="-90"/>
<wire x1="3.8249" y1="2.3366" x2="3.8249" y2="-2.3364" width="0.2032" layer="21"/>
<wire x1="3.8249" y1="-2.3364" x2="3.4911" y2="-2.6702" width="0.2032" layer="21" curve="-90"/>
<wire x1="3.4911" y1="-2.6702" x2="2.8235" y2="-2.6702" width="0.2032" layer="21"/>
<wire x1="-3.481" y1="3.6479" x2="-3.481" y2="2.7419" width="0.2032" layer="21"/>
<wire x1="3.457" y1="3.6241" x2="3.457" y2="2.7419" width="0.2032" layer="21"/>
<wire x1="2.9563" y1="3.6717" x2="2.9563" y2="3.1472" width="0.2032" layer="21"/>
<wire x1="2.9563" y1="3.1472" x2="-2.9803" y2="3.1472" width="0.2032" layer="21"/>
<wire x1="-2.9803" y1="3.1472" x2="-2.9803" y2="3.6241" width="0.2032" layer="21"/>
<wire x1="-3.0042" y1="3.1234" x2="-3.3856" y2="2.7419" width="0.2032" layer="21"/>
<wire x1="2.9802" y1="3.1234" x2="3.314" y2="2.7658" width="0.2032" layer="21"/>
<wire x1="-2.5853" y1="-1.7165" x2="-2.5853" y2="1.4544" width="0.2032" layer="21"/>
<wire x1="-2.5853" y1="1.4544" x2="2.6089" y2="1.4544" width="0.2032" layer="21"/>
<wire x1="2.6089" y1="1.4544" x2="2.6089" y2="-1.7404" width="0.2032" layer="21"/>
<pad name="1" x="-1.25" y="0" drill="1" diameter="1.8" shape="long" rot="R90"/>
<pad name="2" x="1.25" y="0" drill="1" diameter="1.8" shape="long" rot="R90"/>
<text x="-2.54" y="-5.08" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.81" y="4.445" size="1.27" layer="27">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="MKS1852-6-0-202" urn="urn:adsk.eagle:package:10067/1" locally_modified="yes" type="box">
<description>Serie MKS 1850 Lötstifte In-line, Raster 2.5 mm, Stiftwannen, stehend, mit Rastlasche
Source: http://www.stocko.de/de/index.php?seite=ka_rfk2_mks1850.html&amp;rubrik=Katalog+-+RFK+2
rfk_1850.pdf</description>
<packageinstances>
<packageinstance name="MKS1852-6-0-202"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="PIN2" urn="urn:adsk.eagle:symbol:10029/1">
<wire x1="-2.54" y1="2.54" x2="5.08" y2="2.54" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-5.08" width="0.254" layer="94"/>
<wire x1="5.08" y1="-5.08" x2="-2.54" y2="-5.08" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-5.08" x2="-2.54" y2="2.54" width="0.254" layer="94"/>
<text x="-2.54" y="3.81" size="1.778" layer="95">&gt;NAME</text>
<text x="-2.54" y="-7.62" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="-5.08" y="0" visible="pin" length="short" direction="in"/>
<pin name="2" x="-5.08" y="-2.54" visible="pin" length="short" direction="in"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="MKS1852-6-0-202" urn="urn:adsk.eagle:component:10084/1" locally_modified="yes" prefix="X">
<description>&lt;b&gt;Serie MKS 1850&lt;/b&gt; Lötstifte In-line, Raster 2.5 mm, Stiftwannen, stehend, mit Rastlasche&lt;p&gt;
Source: http://www.stocko.de/de/index.php?seite=ka_rfk2_mks1850.html&amp;rubrik=Katalog+-+RFK+2&lt;br&gt;
rfk_1850.pdf</description>
<gates>
<gate name="G$1" symbol="PIN2" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MKS1852-6-0-202">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:10067/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="transistor_mod">
<description>&lt;b&gt;Transistors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="TO92-EBC" urn="urn:adsk.eagle:footprint:29852/1" locally_modified="yes">
<description>&lt;b&gt;TO-92&lt;/b&gt; Pads In Line E B C from top&lt;p&gt;</description>
<wire x1="-2.095" y1="-2.921" x2="2.095" y2="-2.921" width="0.127" layer="21"/>
<wire x1="-2.413" y1="-0.1341" x2="2.413" y2="-0.1341" width="0.127" layer="21" curve="-129.583"/>
<wire x1="1.136" y1="-1.397" x2="-1.136" y2="-1.397" width="0.127" layer="51"/>
<wire x1="-2.413" y1="-0.1341" x2="-2.664" y2="-1.397" width="0.127" layer="51" curve="27.9376"/>
<wire x1="-2.664" y1="-1.397" x2="-2.413" y2="-2.4059" width="0.127" layer="51" curve="22.4788"/>
<wire x1="-1.404" y1="-1.397" x2="-2.664" y2="-1.397" width="0.1524" layer="51"/>
<wire x1="-2.4135" y1="-2.4059" x2="-2.095" y2="-2.921" width="0.127" layer="21" curve="13.0385"/>
<wire x1="-1.136" y1="-1.397" x2="-1.404" y2="-1.397" width="0.127" layer="21"/>
<wire x1="2.413" y1="-2.4059" x2="2.664" y2="-1.397" width="0.127" layer="51" curve="22.4788"/>
<wire x1="2.664" y1="-1.397" x2="2.413" y2="-0.1341" width="0.127" layer="51" curve="27.9376"/>
<wire x1="2.664" y1="-1.397" x2="1.404" y2="-1.397" width="0.127" layer="51"/>
<wire x1="1.404" y1="-1.397" x2="1.136" y2="-1.397" width="0.127" layer="21"/>
<wire x1="2.095" y1="-2.921" x2="2.4247" y2="-2.3818" width="0.127" layer="21" curve="13.6094"/>
<pad name="C" x="2.54" y="-1.27" drill="0.8128" diameter="1.6764" shape="long" rot="R90"/>
<pad name="E" x="-2.54" y="-1.27" drill="0.8128" diameter="1.6764" shape="long" rot="R90"/>
<pad name="B" x="0" y="-1.27" drill="0.8128" diameter="1.6764" shape="long" rot="R90"/>
<text x="-2.54" y="1.778" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-4.572" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="TO3" urn="urn:adsk.eagle:footprint:29823/1">
<description>&lt;b&gt;TO-3&lt;/b&gt;</description>
<wire x1="6.3754" y1="-10.9982" x2="17.3736" y2="-4.5212" width="0.1524" layer="21"/>
<wire x1="5.969" y1="11.2014" x2="17.3736" y2="4.5212" width="0.1524" layer="21"/>
<wire x1="-6.35" y1="10.9982" x2="-17.272" y2="4.572" width="0.1524" layer="21"/>
<wire x1="-5.9436" y1="-11.2268" x2="-17.2466" y2="-4.5974" width="0.1524" layer="21"/>
<wire x1="17.3366" y1="4.545" x2="19.812" y2="0" width="0.1524" layer="21" curve="-57.1487"/>
<wire x1="17.3366" y1="-4.545" x2="19.812" y2="0" width="0.1524" layer="21" curve="57.1487"/>
<wire x1="-19.812" y1="0" x2="-17.0927" y2="4.6935" width="0.1524" layer="21" curve="-60.1731"/>
<wire x1="-19.812" y1="0" x2="-17.1555" y2="-4.657" width="0.1524" layer="21" curve="59.4042"/>
<wire x1="0" y1="-12.7" x2="6.3718" y2="-10.9859" width="0.1524" layer="21" curve="30.1136"/>
<wire x1="-5.9765" y1="-11.2059" x2="0" y2="-12.7" width="0.1524" layer="21" curve="28.0726"/>
<wire x1="-6.3718" y1="10.9859" x2="0" y2="12.7" width="0.1524" layer="21" curve="-30.1136"/>
<wire x1="0" y1="12.7" x2="5.9765" y2="11.2059" width="0.1524" layer="21" curve="-28.0726"/>
<circle x="0" y="0" radius="9.3726" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="11.684" width="0.0508" layer="21"/>
<circle x="15.113" y="0" radius="2.159" width="0.1524" layer="51"/>
<circle x="-15.113" y="0" radius="2.159" width="0.1524" layer="51"/>
<pad name="E" x="-1.778" y="-5.461" drill="1.1938" diameter="3.1496" shape="long"/>
<pad name="B" x="-1.778" y="5.461" drill="1.1938" diameter="3.1496" shape="long"/>
<pad name="C" x="15.113" y="0" drill="4.1148" diameter="6.477"/>
<pad name="C/" x="-15.113" y="0" drill="4.1148" diameter="6.477"/>
<text x="-5.08" y="1.27" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
<text x="15.24" y="-4.953" size="1.27" layer="51" ratio="10" rot="R90">C</text>
<text x="-3.81" y="-6.223" size="1.27" layer="51" ratio="10" rot="R90">E</text>
<text x="-3.81" y="4.572" size="1.27" layer="51" ratio="10" rot="R90">B</text>
</package>
<package name="TO18" urn="urn:adsk.eagle:footprint:28463/1">
<description>&lt;b&gt;TO 18&lt;/b&gt;</description>
<wire x1="0.9289" y1="2.227" x2="2.227" y2="0.9289" width="0.0508" layer="51" curve="-44.7172"/>
<wire x1="0.9289" y1="-2.227" x2="2.227" y2="-0.9289" width="0.0508" layer="51" curve="44.7172"/>
<wire x1="-3.937" y1="-0.508" x2="-3.937" y2="0.508" width="0.127" layer="21"/>
<wire x1="-2.227" y1="-0.9289" x2="0.929" y2="2.2271" width="0.0508" layer="21" curve="-135.281"/>
<wire x1="-2.227" y1="-0.9289" x2="-0.9289" y2="-2.227" width="0.0508" layer="51" curve="44.7172"/>
<wire x1="-3.937" y1="-0.508" x2="-2.8765" y2="-0.508" width="0.127" layer="21"/>
<wire x1="-0.9289" y1="-2.227" x2="0.9289" y2="-2.227" width="0.0508" layer="21" curve="45.2828"/>
<wire x1="-3.937" y1="0.508" x2="-2.8765" y2="0.508" width="0.127" layer="21"/>
<wire x1="2.227" y1="-0.9289" x2="2.227" y2="0.9289" width="0.0508" layer="21" curve="45.2828"/>
<circle x="0" y="0" radius="2.921" width="0.127" layer="21"/>
<pad name="1" x="-1.27" y="-1.27" drill="0.8128" shape="octagon"/>
<pad name="2" x="1.27" y="-1.27" drill="0.8128" shape="octagon"/>
<pad name="3" x="1.27" y="1.27" drill="0.8128" shape="octagon"/>
<text x="3.175" y="0" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="3.175" y="-1.905" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-1.905" y="-1.27" size="1.27" layer="51" ratio="10">1</text>
<text x="0.635" y="-1.27" size="1.27" layer="51" ratio="10">2</text>
<text x="0" y="0.635" size="1.27" layer="51" ratio="10">3</text>
</package>
<package name="TO18-" urn="urn:adsk.eagle:footprint:28930/1">
<description>&lt;b&gt;TO-18&lt;/b&gt;</description>
<wire x1="-2.2098" y1="-0.9692" x2="2.2098" y2="-0.9692" width="0.0508" layer="21" curve="132.636"/>
<wire x1="-0.9692" y1="2.2098" x2="0.9692" y2="2.2098" width="0.0508" layer="51" curve="-47.3637"/>
<wire x1="-2.2098" y1="0.9692" x2="-2.2098" y2="-0.9692" width="0.0508" layer="51" curve="47.3637"/>
<wire x1="-2.2098" y1="0.9692" x2="-0.9692" y2="2.2098" width="0.0508" layer="21" curve="-42.6363"/>
<wire x1="2.2098" y1="-0.9692" x2="2.2098" y2="0.9692" width="0.0508" layer="51" curve="47.3637"/>
<wire x1="1.649" y1="-2.411" x2="2.413" y2="-3.175" width="0.1524" layer="21"/>
<wire x1="2.411" y1="-1.649" x2="3.175" y2="-2.413" width="0.1524" layer="21"/>
<wire x1="2.413" y1="-3.175" x2="3.175" y2="-2.413" width="0.1524" layer="21"/>
<wire x1="0.9692" y1="2.2098" x2="2.2098" y2="0.9692" width="0.0508" layer="21" curve="-42.6363"/>
<circle x="0" y="0" radius="2.921" width="0.1524" layer="21"/>
<pad name="1" x="1.905" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="0" y="1.905" drill="0.8128" shape="octagon"/>
<pad name="3" x="-1.905" y="0" drill="0.8128" shape="octagon"/>
<text x="3.302" y="0.381" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="3.302" y="-1.27" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="1.016" y="-1.143" size="1.27" layer="51" ratio="10">1</text>
<text x="-0.508" y="0.635" size="1.27" layer="51" ratio="10">2</text>
<text x="-1.905" y="-1.27" size="1.27" layer="51" ratio="10">3</text>
</package>
<package name="TO202" urn="urn:adsk.eagle:footprint:28460/1">
<description>&lt;b&gt;TO 202 horizontal&lt;/b&gt;</description>
<wire x1="-5.207" y1="-1.27" x2="-5.207" y2="7.3406" width="0.127" layer="21"/>
<wire x1="-3.5052" y1="7.3406" x2="-3.5052" y2="8.8138" width="0.127" layer="21"/>
<wire x1="-3.5052" y1="8.8138" x2="-5.207" y2="8.8138" width="0.127" layer="21"/>
<wire x1="5.207" y1="8.8138" x2="3.5052" y2="8.8138" width="0.127" layer="21"/>
<wire x1="3.5052" y1="8.8138" x2="3.5052" y2="7.3406" width="0.127" layer="21"/>
<wire x1="5.2578" y1="-1.27" x2="5.2578" y2="7.3406" width="0.127" layer="21"/>
<wire x1="-5.207" y1="7.3406" x2="-3.5052" y2="7.3406" width="0.127" layer="21"/>
<wire x1="3.8354" y1="-1.27" x2="3.8354" y2="7.3406" width="0.127" layer="21"/>
<wire x1="-5.207" y1="-1.27" x2="3.8354" y2="-1.27" width="0.127" layer="21"/>
<wire x1="5.207" y1="21.717" x2="5.207" y2="8.8138" width="0.127" layer="21"/>
<wire x1="5.207" y1="21.717" x2="-5.207" y2="21.717" width="0.127" layer="21"/>
<wire x1="-5.207" y1="21.717" x2="-5.207" y2="8.8138" width="0.127" layer="21"/>
<wire x1="-3.5052" y1="7.3406" x2="3.5052" y2="7.3406" width="0.127" layer="21"/>
<wire x1="3.8354" y1="-1.27" x2="5.2578" y2="-1.27" width="0.127" layer="21"/>
<wire x1="3.8354" y1="7.3406" x2="5.2578" y2="7.3406" width="0.127" layer="21"/>
<wire x1="3.5052" y1="7.3406" x2="3.8354" y2="7.3406" width="0.127" layer="21"/>
<wire x1="-4.953" y1="-1.016" x2="4.953" y2="-1.016" width="0.0508" layer="21"/>
<wire x1="4.953" y1="7.112" x2="4.953" y2="-1.016" width="0.0508" layer="21"/>
<wire x1="4.953" y1="7.112" x2="-4.953" y2="7.112" width="0.0508" layer="21"/>
<wire x1="-4.953" y1="-1.016" x2="-4.953" y2="7.112" width="0.0508" layer="21"/>
<circle x="-3.7592" y="-0.2286" radius="0.4318" width="0" layer="21"/>
<circle x="0" y="17.78" radius="1.651" width="0.127" layer="21"/>
<circle x="0" y="17.78" radius="4.191" width="0" layer="42"/>
<circle x="0" y="17.78" radius="4.191" width="0" layer="43"/>
<pad name="1" x="-2.54" y="-5.08" drill="1.1176" shape="long" rot="R90"/>
<pad name="2" x="0" y="-5.08" drill="1.1176" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="-5.08" drill="1.1176" shape="long" rot="R90"/>
<text x="-2.54" y="4.318" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.445" y="2.159" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-2.54" y="-0.635" size="1.27" layer="21" ratio="10">1</text>
<text x="-0.635" y="-0.635" size="1.27" layer="21" ratio="10">2</text>
<text x="1.905" y="-0.635" size="1.27" layer="21" ratio="10">3</text>
<rectangle x1="-0.635" y1="-2.54" x2="0.635" y2="-1.27" layer="21"/>
<rectangle x1="1.905" y1="-2.54" x2="3.175" y2="-1.27" layer="21"/>
<rectangle x1="-3.175" y1="-2.54" x2="-1.905" y2="-1.27" layer="21"/>
<rectangle x1="2.159" y1="-3.429" x2="2.921" y2="-2.54" layer="21"/>
<rectangle x1="2.159" y1="-5.08" x2="2.921" y2="-3.429" layer="51"/>
<rectangle x1="-0.381" y1="-3.429" x2="0.381" y2="-2.54" layer="21"/>
<rectangle x1="-2.921" y1="-3.429" x2="-2.159" y2="-2.54" layer="21"/>
<rectangle x1="-0.381" y1="-5.08" x2="0.381" y2="-3.429" layer="51"/>
<rectangle x1="-2.921" y1="-5.08" x2="-2.159" y2="-3.429" layer="51"/>
<hole x="0" y="17.78" drill="3.302"/>
</package>
<package name="TO202V" urn="urn:adsk.eagle:footprint:28675/1">
<description>&lt;b&gt;TO 202 vertical&lt;/b&gt;</description>
<wire x1="-4.826" y1="-4.7498" x2="2.794" y2="-4.7498" width="0.127" layer="21"/>
<wire x1="-4.826" y1="-0.127" x2="4.8006" y2="-0.127" width="0.127" layer="21"/>
<wire x1="2.794" y1="-4.7498" x2="4.8006" y2="-2.794" width="0.127" layer="21"/>
<wire x1="-4.826" y1="-4.7498" x2="-4.826" y2="-0.127" width="0.127" layer="21"/>
<wire x1="4.8006" y1="-0.127" x2="4.8006" y2="-2.794" width="0.127" layer="21"/>
<circle x="-4.064" y="-3.9878" radius="0.4318" width="0.127" layer="21"/>
<pad name="1" x="-2.54" y="-2.54" drill="1.1176" shape="long" rot="R90"/>
<pad name="2" x="0" y="-2.54" drill="1.1176" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="-2.54" drill="1.1176" shape="long" rot="R90"/>
<text x="-4.445" y="-6.985" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.445" y="-8.89" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-3.175" y="-1.905" size="1.27" layer="51" ratio="10">1</text>
<text x="-0.635" y="-1.905" size="1.27" layer="51" ratio="10">2</text>
<text x="1.905" y="-1.905" size="1.27" layer="51" ratio="10">3</text>
<rectangle x1="3.429" y1="-2.794" x2="4.826" y2="-2.286" layer="21"/>
<rectangle x1="0.889" y1="-2.794" x2="1.651" y2="-2.286" layer="21"/>
<rectangle x1="-1.651" y1="-2.794" x2="-0.889" y2="-2.286" layer="21"/>
<rectangle x1="-4.826" y1="-2.794" x2="-3.429" y2="-2.286" layer="21"/>
<rectangle x1="-3.429" y1="-2.794" x2="-1.651" y2="-2.286" layer="51"/>
<rectangle x1="-0.889" y1="-2.794" x2="0.889" y2="-2.286" layer="51"/>
<rectangle x1="1.651" y1="-2.794" x2="3.429" y2="-2.286" layer="51"/>
</package>
<package name="SOT93" urn="urn:adsk.eagle:footprint:29366/1">
<description>SOT-93&lt;p&gt;
grid 5.45 mm</description>
<wire x1="-7.62" y1="-6.35" x2="7.62" y2="-6.35" width="0.1524" layer="21"/>
<wire x1="7.62" y1="6.223" x2="7.62" y2="-6.35" width="0.1524" layer="21"/>
<wire x1="7.62" y1="6.223" x2="6.985" y2="6.223" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="-6.35" x2="-7.62" y2="6.223" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="6.223" x2="-6.985" y2="6.985" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="6.223" x2="-7.62" y2="6.223" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="6.985" x2="-7.62" y2="6.985" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="6.985" x2="-7.62" y2="9.525" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="9.525" x2="-3.1242" y2="13.3096" width="0.1524" layer="21"/>
<wire x1="6.985" y1="6.223" x2="6.985" y2="6.985" width="0.1524" layer="21"/>
<wire x1="6.985" y1="6.223" x2="-6.985" y2="6.223" width="0.1524" layer="21"/>
<wire x1="6.985" y1="6.985" x2="7.62" y2="6.985" width="0.1524" layer="21"/>
<wire x1="7.62" y1="6.985" x2="7.62" y2="9.525" width="0.1524" layer="21"/>
<wire x1="-6.985" y1="-5.715" x2="6.985" y2="-5.715" width="0.0508" layer="21"/>
<wire x1="6.985" y1="5.715" x2="6.985" y2="-5.715" width="0.0508" layer="21"/>
<wire x1="6.985" y1="5.715" x2="-6.985" y2="5.715" width="0.0508" layer="21"/>
<wire x1="-6.985" y1="-5.715" x2="-6.985" y2="5.715" width="0.0508" layer="21"/>
<wire x1="0" y1="14.605" x2="3.1431" y2="13.3031" width="0.1524" layer="21" curve="-45.0001"/>
<wire x1="-3.1431" y1="13.3031" x2="0" y2="14.605" width="0.1524" layer="21" curve="-45.0001"/>
<wire x1="7.62" y1="9.525" x2="3.1242" y2="13.335" width="0.1524" layer="21"/>
<wire x1="6.985" y1="-5.715" x2="7.62" y2="-6.35" width="0.0508" layer="51"/>
<wire x1="6.985" y1="5.715" x2="7.62" y2="6.35" width="0.0508" layer="51"/>
<wire x1="-6.985" y1="5.715" x2="-7.62" y2="6.35" width="0.0508" layer="51"/>
<wire x1="-6.985" y1="-5.715" x2="-7.62" y2="-6.35" width="0.0508" layer="51"/>
<circle x="0" y="10.16" radius="2.159" width="0.1524" layer="21"/>
<circle x="0" y="10.16" radius="5.08" width="0" layer="42"/>
<circle x="0" y="10.16" radius="5.08" width="0" layer="43"/>
<pad name="B" x="-5.461" y="-10.16" drill="1.397" shape="long" rot="R90"/>
<pad name="C" x="0" y="-10.16" drill="1.397" shape="long" rot="R90"/>
<pad name="E" x="5.461" y="-10.16" drill="1.397" shape="long" rot="R90"/>
<text x="-5.08" y="2.54" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="0" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
<text x="-6.477" y="6.604" size="1.016" layer="21" ratio="10">A20,3mm</text>
<rectangle x1="-6.096" y1="-7.366" x2="-4.318" y2="-6.35" layer="21"/>
<rectangle x1="4.318" y1="-7.366" x2="6.096" y2="-6.35" layer="21"/>
<rectangle x1="-1.143" y1="-7.366" x2="1.143" y2="-6.35" layer="21"/>
<rectangle x1="-6.096" y1="-8.128" x2="-4.826" y2="-6.35" layer="21"/>
<rectangle x1="-0.635" y1="-8.128" x2="0.635" y2="-6.35" layer="21"/>
<rectangle x1="4.826" y1="-8.128" x2="6.096" y2="-6.35" layer="21"/>
<rectangle x1="-6.096" y1="-10.287" x2="-4.826" y2="-8.128" layer="51"/>
<rectangle x1="-0.635" y1="-10.287" x2="0.635" y2="-8.128" layer="51"/>
<rectangle x1="4.826" y1="-10.287" x2="6.096" y2="-8.128" layer="51"/>
<hole x="0" y="10.16" drill="4.1148"/>
</package>
<package name="SOT93V" urn="urn:adsk.eagle:footprint:28676/1">
<description>SOT-93&lt;p&gt;
grid 5.45 mm</description>
<wire x1="7.62" y1="-2.032" x2="3.429" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="3.429" y1="-2.032" x2="2.032" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="2.032" y1="-2.032" x2="-2.032" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="-2.032" x2="-3.429" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-2.032" x2="-7.62" y2="-2.032" width="0.1524" layer="51"/>
<wire x1="-7.62" y1="0" x2="7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-7.239" y1="-5.08" x2="7.239" y2="-5.08" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="-2.032" x2="-7.239" y2="-5.08" width="0.1524" layer="21"/>
<wire x1="7.62" y1="0" x2="7.62" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="7.62" y1="-2.032" x2="7.239" y2="-5.08" width="0.1524" layer="21"/>
<pad name="B" x="-5.461" y="-2.54" drill="1.397" shape="long" rot="R90"/>
<pad name="C" x="0" y="-2.54" drill="1.397" shape="long" rot="R90"/>
<pad name="E" x="5.461" y="-2.54" drill="1.397" shape="long" rot="R90"/>
<text x="-1.397" y="-6.858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-7.62" y="-6.858" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<rectangle x1="-7.62" y1="-2.032" x2="7.62" y2="0" layer="51"/>
</package>
<package name="TO126" urn="urn:adsk.eagle:footprint:29831/1">
<description>&lt;b&gt;TO 126 horizontal&lt;/b&gt;</description>
<wire x1="-3.937" y1="-1.27" x2="-3.937" y2="9.144" width="0.127" layer="21"/>
<wire x1="-3.937" y1="9.144" x2="3.937" y2="9.144" width="0.127" layer="21"/>
<wire x1="3.937" y1="9.144" x2="3.937" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-3.937" y1="-1.27" x2="3.937" y2="-1.27" width="0.127" layer="21"/>
<wire x1="1.2393" y1="3.1591" x2="1.9493" y2="3.8858" width="0.127" layer="21" curve="25.6775"/>
<wire x1="0.7687" y1="3.6188" x2="1.2393" y2="3.1591" width="0.127" layer="21"/>
<wire x1="-0.7687" y1="3.6188" x2="-1.2393" y2="3.1591" width="0.127" layer="21"/>
<wire x1="-1.4787" y1="4.3456" x2="-1.9493" y2="3.8858" width="0.127" layer="21"/>
<wire x1="-1.9493" y1="3.8858" x2="-1.2393" y2="3.1591" width="0.127" layer="21" curve="25.6775"/>
<wire x1="0.508" y1="6.6509" x2="0.508" y2="7.3088" width="0.127" layer="21"/>
<wire x1="-0.508" y1="6.6509" x2="-0.508" y2="7.3088" width="0.127" layer="21"/>
<wire x1="-0.508" y1="7.3088" x2="0.508" y2="7.3088" width="0.127" layer="21" curve="-25.6796"/>
<wire x1="1.4787" y1="4.3456" x2="1.9493" y2="3.8858" width="0.127" layer="21"/>
<circle x="-3.1242" y="-0.4826" radius="0.4318" width="0" layer="21"/>
<circle x="0" y="5.08" radius="3.81" width="0" layer="42"/>
<circle x="0" y="5.08" radius="3.81" width="0" layer="43"/>
<circle x="0" y="5.08" radius="1.651" width="0.127" layer="21"/>
<pad name="1" x="-2.286" y="-5.08" drill="1.1176" shape="long" rot="R90"/>
<pad name="2" x="0" y="-5.08" drill="1.1176" shape="long" rot="R90"/>
<pad name="3" x="2.286" y="-5.08" drill="1.1176" shape="long" rot="R90"/>
<text x="-2.54" y="7.62" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="1.397" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-2.54" y="-0.889" size="1.27" layer="51" ratio="10">1</text>
<text x="-0.508" y="-0.889" size="1.27" layer="51" ratio="10">2</text>
<text x="1.778" y="-0.889" size="1.27" layer="51" ratio="10">3</text>
<rectangle x1="-0.635" y1="-3.048" x2="0.635" y2="-1.27" layer="21"/>
<rectangle x1="-2.667" y1="-3.429" x2="-1.905" y2="-1.27" layer="21"/>
<rectangle x1="-2.667" y1="-5.207" x2="-1.905" y2="-3.429" layer="51"/>
<rectangle x1="-0.381" y1="-3.429" x2="0.381" y2="-1.27" layer="21"/>
<rectangle x1="-0.381" y1="-5.207" x2="0.381" y2="-3.429" layer="51"/>
<rectangle x1="1.905" y1="-3.429" x2="2.667" y2="-1.27" layer="21"/>
<rectangle x1="1.905" y1="-5.207" x2="2.667" y2="-3.429" layer="51"/>
<hole x="0" y="5.08" drill="3.302"/>
</package>
<package name="TO126V" urn="urn:adsk.eagle:footprint:28677/1">
<description>&lt;b&gt;TO 126 vertical&lt;/b&gt;</description>
<wire x1="-3.937" y1="-0.127" x2="-3.937" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-3.937" y1="-1.27" x2="-3.683" y2="-2.794" width="0.127" layer="21"/>
<wire x1="3.683" y1="-2.794" x2="3.937" y2="-1.27" width="0.127" layer="21"/>
<wire x1="3.937" y1="-1.27" x2="3.937" y2="-0.127" width="0.127" layer="21"/>
<wire x1="-3.683" y1="-2.794" x2="-2.794" y2="-2.794" width="0.127" layer="21"/>
<wire x1="-2.794" y1="-2.794" x2="-1.778" y2="-2.794" width="0.127" layer="51"/>
<wire x1="-1.778" y1="-2.794" x2="-0.508" y2="-2.794" width="0.127" layer="21"/>
<wire x1="-0.508" y1="-2.794" x2="0.508" y2="-2.794" width="0.127" layer="51"/>
<wire x1="0.508" y1="-2.794" x2="1.778" y2="-2.794" width="0.127" layer="21"/>
<wire x1="1.778" y1="-2.794" x2="2.794" y2="-2.794" width="0.127" layer="51"/>
<wire x1="2.794" y1="-2.794" x2="3.683" y2="-2.794" width="0.127" layer="21"/>
<circle x="-3.175" y="-2.159" radius="0.4064" width="0.127" layer="51"/>
<pad name="1" x="-2.286" y="-1.27" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="0" y="-1.27" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="2.286" y="-1.27" drill="1.016" shape="long" rot="R90"/>
<text x="-3.9624" y="-4.5466" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.9878" y="-6.3246" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-2.54" y="-1.905" size="1.27" layer="51" ratio="10">1</text>
<text x="-0.381" y="-1.905" size="1.27" layer="51" ratio="10">2</text>
<text x="2.032" y="-1.905" size="1.27" layer="51" ratio="10">3</text>
<rectangle x1="-3.937" y1="-0.381" x2="-3.175" y2="0" layer="21"/>
<rectangle x1="-1.397" y1="-0.381" x2="-0.889" y2="0" layer="21"/>
<rectangle x1="0.889" y1="-0.381" x2="1.397" y2="0" layer="21"/>
<rectangle x1="3.175" y1="-0.381" x2="3.937" y2="0" layer="21"/>
<rectangle x1="-3.175" y1="-0.381" x2="-1.397" y2="0" layer="51"/>
<rectangle x1="-0.889" y1="-0.381" x2="0.889" y2="0" layer="51"/>
<rectangle x1="1.397" y1="-0.381" x2="3.175" y2="0" layer="51"/>
</package>
<package name="TO218" urn="urn:adsk.eagle:footprint:28455/1">
<description>&lt;b&gt;TO 218 horizontal&lt;/b&gt;</description>
<wire x1="-7.62" y1="-6.35" x2="7.62" y2="-6.35" width="0.127" layer="21"/>
<wire x1="7.62" y1="6.223" x2="7.62" y2="0" width="0.127" layer="21"/>
<wire x1="7.62" y1="0" x2="7.62" y2="-3.81" width="0.127" layer="21"/>
<wire x1="7.62" y1="-3.81" x2="7.62" y2="-6.35" width="0.127" layer="21"/>
<wire x1="-7.62" y1="-6.35" x2="-7.62" y2="-3.81" width="0.127" layer="21"/>
<wire x1="-7.62" y1="-3.81" x2="-7.62" y2="0" width="0.127" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="6.223" width="0.127" layer="21"/>
<wire x1="7.62" y1="6.223" x2="-7.62" y2="6.223" width="0.127" layer="21"/>
<wire x1="6.985" y1="5.715" x2="-6.985" y2="5.715" width="0.0508" layer="21"/>
<wire x1="6.985" y1="0.508" x2="6.985" y2="5.715" width="0.0508" layer="21"/>
<wire x1="-6.985" y1="-5.715" x2="6.985" y2="-5.715" width="0.0508" layer="21"/>
<wire x1="7.62" y1="12.065" x2="5.08" y2="14.605" width="0.127" layer="21"/>
<wire x1="7.62" y1="12.065" x2="7.62" y2="6.223" width="0.127" layer="21"/>
<wire x1="5.08" y1="14.605" x2="-5.08" y2="14.605" width="0.127" layer="21"/>
<wire x1="-7.62" y1="12.065" x2="-5.08" y2="14.605" width="0.127" layer="21"/>
<wire x1="-7.62" y1="12.065" x2="-7.62" y2="6.223" width="0.127" layer="21"/>
<wire x1="7.62" y1="0" x2="7.62" y2="-3.81" width="0.127" layer="21" curve="180"/>
<wire x1="6.985" y1="-5.715" x2="6.985" y2="-4.318" width="0.0508" layer="21"/>
<wire x1="6.9522" y1="0.4991" x2="6.9851" y2="-4.3178" width="0.0508" layer="21" curve="149.733"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-3.81" width="0.127" layer="21" curve="-180"/>
<wire x1="-6.985" y1="-5.715" x2="-6.985" y2="-4.318" width="0.0508" layer="21"/>
<wire x1="-6.985" y1="0.508" x2="-6.985" y2="5.715" width="0.0508" layer="21"/>
<wire x1="-6.985" y1="-4.318" x2="-6.985" y2="0.508" width="0.0508" layer="21" curve="150.513"/>
<circle x="0" y="10.16" radius="2.159" width="0.127" layer="21"/>
<circle x="0" y="10.16" radius="5.08" width="0" layer="42"/>
<circle x="0" y="10.16" radius="5.08" width="0" layer="43"/>
<pad name="1" x="-5.461" y="-10.16" drill="1.397" shape="long" rot="R90"/>
<pad name="2" x="0" y="-10.16" drill="1.397" shape="long" rot="R90"/>
<pad name="3" x="5.461" y="-10.16" drill="1.397" shape="long" rot="R90"/>
<text x="-5.08" y="2.54" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="0" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
<text x="-5.08" y="-5.08" size="1.27" layer="21" ratio="10">1</text>
<text x="-0.635" y="-5.08" size="1.27" layer="21" ratio="10">2</text>
<text x="4.445" y="-5.08" size="1.27" layer="21" ratio="10">3</text>
<rectangle x1="-6.096" y1="-7.366" x2="-4.318" y2="-6.35" layer="21"/>
<rectangle x1="4.318" y1="-7.366" x2="6.096" y2="-6.35" layer="21"/>
<rectangle x1="-1.143" y1="-7.366" x2="1.143" y2="-6.35" layer="21"/>
<rectangle x1="-6.096" y1="-8.128" x2="-4.826" y2="-6.35" layer="21"/>
<rectangle x1="-6.096" y1="-10.287" x2="-4.826" y2="-8.128" layer="51"/>
<rectangle x1="-0.635" y1="-8.128" x2="0.635" y2="-6.35" layer="21"/>
<rectangle x1="4.826" y1="-8.128" x2="6.096" y2="-6.35" layer="21"/>
<rectangle x1="-0.635" y1="-10.287" x2="0.635" y2="-8.128" layer="51"/>
<rectangle x1="4.826" y1="-10.287" x2="6.096" y2="-8.128" layer="51"/>
<hole x="0" y="10.16" drill="4.1148"/>
</package>
<package name="TO218V" urn="urn:adsk.eagle:footprint:28456/1">
<description>&lt;b&gt;TO 218 vertical&lt;/b&gt;</description>
<wire x1="-6.985" y1="-5.08" x2="6.985" y2="-5.08" width="0.127" layer="21"/>
<wire x1="6.985" y1="-5.08" x2="7.366" y2="-4.699" width="0.127" layer="21"/>
<wire x1="7.366" y1="-4.699" x2="7.62" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-6.985" y1="-5.08" x2="-7.366" y2="-4.699" width="0.127" layer="21"/>
<wire x1="-7.366" y1="-4.699" x2="-7.62" y2="-1.524" width="0.127" layer="21"/>
<pad name="1" x="-5.461" y="-3.175" drill="1.397" shape="long" rot="R90"/>
<pad name="2" x="0" y="-3.175" drill="1.397" shape="long" rot="R90"/>
<pad name="3" x="5.461" y="-3.175" drill="1.397" shape="long" rot="R90"/>
<text x="-7.62" y="-6.858" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.016" y="-6.858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-6.096" y="-4.445" size="1.27" layer="51" ratio="10">1</text>
<text x="-0.635" y="-4.445" size="1.27" layer="51" ratio="10">2</text>
<text x="4.826" y="-4.445" size="1.27" layer="51" ratio="10">3</text>
<rectangle x1="-7.874" y1="-1.143" x2="7.874" y2="0" layer="21"/>
<rectangle x1="-3.556" y1="-2.032" x2="-1.905" y2="-1.143" layer="21"/>
<rectangle x1="1.905" y1="-2.032" x2="3.556" y2="-1.143" layer="21"/>
<rectangle x1="-7.874" y1="-2.032" x2="-7.239" y2="-1.143" layer="21"/>
<rectangle x1="7.239" y1="-2.032" x2="7.874" y2="-1.143" layer="21"/>
<rectangle x1="3.556" y1="-2.032" x2="7.239" y2="-1.143" layer="51"/>
<rectangle x1="-7.239" y1="-2.032" x2="-3.556" y2="-1.143" layer="51"/>
<rectangle x1="-1.905" y1="-2.032" x2="1.905" y2="-1.143" layer="51"/>
</package>
<package name="TO220" urn="urn:adsk.eagle:footprint:28453/1">
<description>&lt;b&gt;TO 220 horizontal&lt;/b&gt;</description>
<wire x1="-5.207" y1="-1.27" x2="5.207" y2="-1.27" width="0.127" layer="21"/>
<wire x1="5.207" y1="14.605" x2="-5.207" y2="14.605" width="0.127" layer="21"/>
<wire x1="5.207" y1="-1.27" x2="5.207" y2="11.176" width="0.127" layer="21"/>
<wire x1="5.207" y1="11.176" x2="4.318" y2="11.176" width="0.127" layer="21"/>
<wire x1="4.318" y1="11.176" x2="4.318" y2="12.7" width="0.127" layer="21"/>
<wire x1="4.318" y1="12.7" x2="5.207" y2="12.7" width="0.127" layer="21"/>
<wire x1="5.207" y1="12.7" x2="5.207" y2="14.605" width="0.127" layer="21"/>
<wire x1="-5.207" y1="-1.27" x2="-5.207" y2="11.176" width="0.127" layer="21"/>
<wire x1="-5.207" y1="11.176" x2="-4.318" y2="11.176" width="0.127" layer="21"/>
<wire x1="-4.318" y1="11.176" x2="-4.318" y2="12.7" width="0.127" layer="21"/>
<wire x1="-4.318" y1="12.7" x2="-5.207" y2="12.7" width="0.127" layer="21"/>
<wire x1="-5.207" y1="12.7" x2="-5.207" y2="14.605" width="0.127" layer="21"/>
<wire x1="-4.572" y1="-0.635" x2="4.572" y2="-0.635" width="0.0508" layer="21"/>
<wire x1="4.572" y1="7.62" x2="4.572" y2="-0.635" width="0.0508" layer="21"/>
<wire x1="4.572" y1="7.62" x2="-4.572" y2="7.62" width="0.0508" layer="21"/>
<wire x1="-4.572" y1="-0.635" x2="-4.572" y2="7.62" width="0.0508" layer="21"/>
<circle x="0" y="11.176" radius="1.8034" width="0.127" layer="21"/>
<circle x="0" y="11.176" radius="4.191" width="0" layer="42"/>
<circle x="0" y="11.176" radius="4.191" width="0" layer="43"/>
<pad name="1" x="-2.54" y="-6.35" drill="1.1176" shape="long" rot="R90"/>
<pad name="2" x="0" y="-6.35" drill="1.1176" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="-6.35" drill="1.1176" shape="long" rot="R90"/>
<text x="-3.81" y="5.207" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.937" y="2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
<text x="-4.445" y="7.874" size="0.9906" layer="21" ratio="12">A17,5mm</text>
<text x="-3.175" y="0" size="1.27" layer="51" ratio="10">1</text>
<text x="-0.635" y="0" size="1.27" layer="51" ratio="10">2</text>
<text x="1.905" y="0" size="1.27" layer="51" ratio="10">3</text>
<rectangle x1="2.159" y1="-4.699" x2="2.921" y2="-4.064" layer="21"/>
<rectangle x1="-0.381" y1="-4.699" x2="0.381" y2="-4.064" layer="21"/>
<rectangle x1="-2.921" y1="-4.699" x2="-2.159" y2="-4.064" layer="21"/>
<rectangle x1="-3.175" y1="-4.064" x2="-1.905" y2="-1.27" layer="21"/>
<rectangle x1="-0.635" y1="-4.064" x2="0.635" y2="-1.27" layer="21"/>
<rectangle x1="1.905" y1="-4.064" x2="3.175" y2="-1.27" layer="21"/>
<rectangle x1="-2.921" y1="-6.604" x2="-2.159" y2="-4.699" layer="51"/>
<rectangle x1="-0.381" y1="-6.604" x2="0.381" y2="-4.699" layer="51"/>
<rectangle x1="2.159" y1="-6.604" x2="2.921" y2="-4.699" layer="51"/>
<hole x="0" y="11.176" drill="3.302"/>
</package>
<package name="TO220V" urn="urn:adsk.eagle:footprint:28680/1">
<description>&lt;b&gt;TO 220 vertical&lt;/b&gt;</description>
<wire x1="5.08" y1="-1.143" x2="4.953" y2="-4.064" width="0.127" layer="21"/>
<wire x1="4.699" y1="-4.318" x2="4.953" y2="-4.064" width="0.127" layer="21"/>
<wire x1="4.699" y1="-4.318" x2="-4.699" y2="-4.318" width="0.127" layer="21"/>
<wire x1="-4.953" y1="-4.064" x2="-4.699" y2="-4.318" width="0.127" layer="21"/>
<wire x1="-4.953" y1="-4.064" x2="-5.08" y2="-1.143" width="0.127" layer="21"/>
<circle x="-4.4958" y="-3.7084" radius="0.254" width="0.127" layer="21"/>
<pad name="1" x="-2.54" y="-2.54" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="0" y="-2.54" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="-2.54" drill="1.016" shape="long" rot="R90"/>
<text x="-5.08" y="-6.0452" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-7.62" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-5.334" y1="-0.762" x2="5.334" y2="0" layer="21"/>
<rectangle x1="-5.334" y1="-1.27" x2="-3.429" y2="-0.762" layer="21"/>
<rectangle x1="-1.651" y1="-1.27" x2="-0.889" y2="-0.762" layer="21"/>
<rectangle x1="-3.429" y1="-1.27" x2="-1.651" y2="-0.762" layer="51"/>
<rectangle x1="0.889" y1="-1.27" x2="1.651" y2="-0.762" layer="21"/>
<rectangle x1="3.429" y1="-1.27" x2="5.334" y2="-0.762" layer="21"/>
<rectangle x1="-0.889" y1="-1.27" x2="0.889" y2="-0.762" layer="51"/>
<rectangle x1="1.651" y1="-1.27" x2="3.429" y2="-0.762" layer="51"/>
</package>
<package name="TO225AA" urn="urn:adsk.eagle:footprint:29839/1">
<description>&lt;b&gt;TO-225&lt;/b&gt;&lt;p&gt;
grid 2.54 mm, vertical</description>
<wire x1="-3.4925" y1="-6.985" x2="-3.4925" y2="3.81" width="0.127" layer="21"/>
<wire x1="-3.4925" y1="3.81" x2="3.4925" y2="3.81" width="0.127" layer="21"/>
<wire x1="3.4925" y1="3.81" x2="3.4925" y2="-6.985" width="0.127" layer="21"/>
<wire x1="-3.4925" y1="-6.985" x2="3.4925" y2="-6.985" width="0.127" layer="21"/>
<wire x1="-2.381" y1="-8.096" x2="-2.381" y2="-9.525" width="0.4064" layer="21"/>
<wire x1="0" y1="-7.779" x2="0" y2="-9.525" width="0.4064" layer="21"/>
<wire x1="2.3813" y1="-8.0963" x2="2.381" y2="-9.525" width="0.4064" layer="21"/>
<wire x1="-2.54" y1="3.175" x2="-2.54" y2="-3.334" width="0.127" layer="51"/>
<wire x1="-2.54" y1="-3.334" x2="2.54" y2="-3.334" width="0.127" layer="51"/>
<wire x1="2.54" y1="-3.334" x2="2.54" y2="3.175" width="0.127" layer="51"/>
<wire x1="2.54" y1="3.175" x2="-2.54" y2="3.175" width="0.127" layer="51"/>
<wire x1="-1.949" y1="1.194" x2="-1.2391" y2="1.9206" width="0.127" layer="21" curve="-25.6749"/>
<wire x1="-0.769" y1="1.461" x2="-1.239" y2="1.921" width="0.127" layer="21"/>
<wire x1="0.769" y1="1.461" x2="1.239" y2="1.921" width="0.127" layer="21"/>
<wire x1="1.479" y1="0.734" x2="1.949" y2="1.194" width="0.127" layer="21"/>
<wire x1="1.239" y1="1.921" x2="1.949" y2="1.1944" width="0.127" layer="21" curve="-25.6741"/>
<wire x1="-0.508" y1="-1.571" x2="-0.508" y2="-2.229" width="0.127" layer="21"/>
<wire x1="0.508" y1="-1.571" x2="0.508" y2="-2.229" width="0.127" layer="21"/>
<wire x1="-0.508" y1="-2.229" x2="0.508" y2="-2.229" width="0.127" layer="21" curve="25.6785"/>
<wire x1="-1.479" y1="0.734" x2="-1.949" y2="1.194" width="0.127" layer="21"/>
<circle x="0" y="0" radius="3.81" width="0" layer="42"/>
<circle x="0" y="0" radius="3.81" width="0" layer="43"/>
<circle x="0" y="0" radius="1.651" width="0.127" layer="21"/>
<pad name="1" x="-2.3813" y="-11.43" drill="0.8128" shape="long" rot="R90"/>
<pad name="2" x="0" y="-11.43" drill="0.8128" shape="long" rot="R90"/>
<pad name="3" x="2.3813" y="-11.43" drill="0.8128" shape="long" rot="R90"/>
<text x="-2.6988" y="-6.6675" size="1.27" layer="51">1</text>
<text x="-0.4763" y="-6.6675" size="1.27" layer="51">2</text>
<text x="1.905" y="-6.6675" size="1.27" layer="51">3</text>
<text x="3.0163" y="5.3975" size="1.27" layer="25" ratio="10" rot="R180">&gt;NAME</text>
<text x="3.4925" y="-3.7783" size="1.27" layer="27" ratio="10" rot="R180">&gt;VALUE</text>
<rectangle x1="-2.6988" y1="-8.255" x2="-2.0638" y2="-6.985" layer="21"/>
<rectangle x1="2.0638" y1="-8.255" x2="2.6988" y2="-6.985" layer="21"/>
<rectangle x1="-0.3175" y1="-7.9375" x2="0.3175" y2="-6.985" layer="21"/>
<hole x="0" y="0" drill="3.302"/>
</package>
<package name="TO39" urn="urn:adsk.eagle:footprint:28658/1">
<description>&lt;b&gt;TO-39&lt;/b&gt;</description>
<wire x1="-4.0386" y1="-3.5306" x2="-3.5052" y2="-2.9972" width="0.1524" layer="21"/>
<wire x1="-2.9718" y1="-3.5306" x2="-3.5052" y2="-4.064" width="0.1524" layer="21"/>
<wire x1="-3.5052" y1="-4.064" x2="-4.0386" y2="-3.5306" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="4.572" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="3.8608" width="0.0508" layer="21"/>
<pad name="1" x="0" y="-2.54" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="0" y="2.54" drill="0.8128" shape="octagon"/>
<text x="-2.54" y="5.08" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="TO5" urn="urn:adsk.eagle:footprint:28657/1">
<description>&lt;b&gt;TO 5&lt;/b&gt;</description>
<wire x1="-4.0386" y1="-3.5306" x2="-3.5052" y2="-2.9972" width="0.127" layer="21"/>
<wire x1="-2.9718" y1="-3.5306" x2="-3.5052" y2="-4.064" width="0.127" layer="21"/>
<wire x1="-3.5052" y1="-4.064" x2="-4.0386" y2="-3.5306" width="0.127" layer="21"/>
<circle x="0" y="0" radius="4.572" width="0.127" layer="21"/>
<circle x="0" y="0" radius="3.8608" width="0.0508" layer="21"/>
<pad name="1" x="0" y="-2.54" drill="0.8128" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="octagon"/>
<pad name="3" x="0" y="2.54" drill="0.8128" shape="octagon"/>
<text x="-2.54" y="5.08" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="TO66" urn="urn:adsk.eagle:footprint:29176/1">
<description>&lt;b&gt;TO-66&lt;/b&gt;</description>
<wire x1="-3.501" y1="-7.895" x2="3.5012" y2="-7.895" width="0.1524" layer="21" curve="47.8332"/>
<wire x1="-3.501" y1="7.895" x2="3.5012" y2="7.895" width="0.1524" layer="21" curve="-47.8332"/>
<wire x1="-13.711" y1="3.3666" x2="-13.711" y2="-3.367" width="0.1524" layer="21" curve="132.168"/>
<wire x1="-3.501" y1="-7.895" x2="-13.711" y2="-3.367" width="0.1524" layer="21"/>
<wire x1="-3.501" y1="7.895" x2="-13.711" y2="3.367" width="0.1524" layer="21"/>
<wire x1="13.711" y1="-3.367" x2="13.711" y2="3.3666" width="0.1524" layer="21" curve="132.168"/>
<wire x1="3.501" y1="-7.895" x2="13.711" y2="-3.367" width="0.1524" layer="21"/>
<wire x1="3.501" y1="7.895" x2="13.711" y2="3.367" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="7.874" width="0.0508" layer="21"/>
<circle x="0" y="0" radius="6.35" width="0.1524" layer="21"/>
<circle x="-12.192" y="0" radius="1.905" width="0.1524" layer="51"/>
<circle x="12.192" y="0" radius="1.905" width="0.1524" layer="51"/>
<pad name="TO66" x="-12.192" y="0" drill="3.302" diameter="6.35"/>
<pad name="3" x="12.192" y="0" drill="3.302" diameter="6.35"/>
<pad name="2" x="-2.54" y="2.54" drill="1.1176" diameter="2.54" shape="long"/>
<pad name="1" x="-2.54" y="-2.54" drill="1.1176" diameter="2.54" shape="long"/>
<text x="8.89" y="-4.445" size="1.27" layer="21" ratio="10">3</text>
<text x="1.27" y="-2.54" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="3.175" y="-3.175" size="1.27" layer="27" ratio="10" rot="R90">&gt;VALUE</text>
<text x="-5.08" y="1.905" size="1.27" layer="21" ratio="10">2</text>
<text x="-5.08" y="-3.175" size="1.27" layer="21" ratio="10">1</text>
</package>
<package name="TO92" urn="urn:adsk.eagle:footprint:28459/1">
<description>&lt;b&gt;TO 92&lt;/b&gt;</description>
<wire x1="-2.0946" y1="-1.651" x2="-2.6549" y2="-0.254" width="0.127" layer="21" curve="-32.781"/>
<wire x1="-2.6549" y1="-0.254" x2="-0.7863" y2="2.5485" width="0.127" layer="21" curve="-78.3185"/>
<wire x1="0.7863" y1="2.5484" x2="2.0945" y2="-1.651" width="0.127" layer="21" curve="-111.1"/>
<wire x1="-2.0945" y1="-1.651" x2="2.0945" y2="-1.651" width="0.127" layer="21"/>
<wire x1="-2.2537" y1="-0.254" x2="-0.2863" y2="-0.254" width="0.127" layer="51"/>
<wire x1="-2.6549" y1="-0.254" x2="-2.2537" y2="-0.254" width="0.127" layer="21"/>
<wire x1="-0.2863" y1="-0.254" x2="0.2863" y2="-0.254" width="0.127" layer="21"/>
<wire x1="2.2537" y1="-0.254" x2="2.6549" y2="-0.254" width="0.127" layer="21"/>
<wire x1="0.2863" y1="-0.254" x2="2.2537" y2="-0.254" width="0.127" layer="51"/>
<wire x1="-0.7863" y1="2.5485" x2="0.7863" y2="2.5485" width="0.127" layer="51" curve="-34.2936"/>
<pad name="1" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="0" y="1.905" drill="0.8128" shape="octagon"/>
<pad name="3" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="3.175" y="0.635" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="3.175" y="-1.27" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-0.635" y="0.635" size="1.27" layer="51" ratio="10">2</text>
<text x="-2.159" y="0" size="1.27" layer="51" ratio="10">3</text>
<text x="1.143" y="0" size="1.27" layer="51" ratio="10">1</text>
</package>
<package name="TO92-E1" urn="urn:adsk.eagle:footprint:29853/1">
<description>&lt;b&gt;TO-92&lt;/b&gt; Pads Triangle Reverse</description>
<wire x1="-2.095" y1="-1.651" x2="-0.7869" y2="2.5484" width="0.127" layer="21" curve="-111.098"/>
<wire x1="0.7869" y1="2.5484" x2="2.095" y2="-1.651" width="0.127" layer="21" curve="-111.098"/>
<wire x1="-2.095" y1="-1.651" x2="2.095" y2="-1.651" width="0.127" layer="21"/>
<wire x1="-2.254" y1="-0.254" x2="-0.286" y2="-0.254" width="0.127" layer="51"/>
<wire x1="-2.655" y1="-0.254" x2="-2.254" y2="-0.254" width="0.127" layer="21"/>
<wire x1="-0.286" y1="-0.254" x2="0.286" y2="-0.254" width="0.127" layer="21"/>
<wire x1="2.254" y1="-0.254" x2="2.655" y2="-0.254" width="0.127" layer="21"/>
<wire x1="0.286" y1="-0.254" x2="2.254" y2="-0.254" width="0.127" layer="51"/>
<wire x1="-0.7864" y1="2.5484" x2="0.7864" y2="2.5484" width="0.127" layer="51" curve="-34.299"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" shape="octagon"/>
<pad name="2" x="0" y="1.905" drill="0.8128" shape="octagon"/>
<pad name="3" x="1.27" y="0" drill="0.8128" shape="octagon"/>
<text x="3.175" y="0.635" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="3.175" y="-1.27" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="TO92L" urn="urn:adsk.eagle:footprint:29854/1">
<description>&lt;b&gt;TO-92&lt;/b&gt;&lt;p&gt;
grid 5.08 mm</description>
<wire x1="-1.1359" y1="2.413" x2="-0.127" y2="2.664" width="0.1524" layer="51" curve="-22.4788"/>
<wire x1="-0.127" y1="2.664" x2="1.1359" y2="2.413" width="0.1524" layer="51" curve="-27.9376"/>
<wire x1="-1.1359" y1="-2.413" x2="-0.127" y2="-2.664" width="0.1524" layer="51" curve="22.4788"/>
<wire x1="-0.127" y1="-2.664" x2="1.1359" y2="-2.413" width="0.1524" layer="51" curve="27.9376"/>
<wire x1="-1.651" y1="2.095" x2="-1.651" y2="-2.095" width="0.1524" layer="21"/>
<wire x1="-0.127" y1="-1.404" x2="-0.127" y2="1.404" width="0.1524" layer="21"/>
<wire x1="-0.127" y1="-2.664" x2="-0.127" y2="-1.404" width="0.1524" layer="51"/>
<wire x1="-1.651" y1="-2.095" x2="-1.1118" y2="-2.4247" width="0.1524" layer="21" curve="13.6094"/>
<wire x1="-0.127" y1="1.404" x2="-0.127" y2="2.664" width="0.1524" layer="51"/>
<wire x1="-1.651" y1="2.095" x2="-1.1359" y2="2.4135" width="0.1524" layer="21" curve="-13.0385"/>
<wire x1="2.413" y1="-1.1359" x2="2.413" y2="1.1359" width="0.1524" layer="51" curve="50.4167"/>
<wire x1="1.1359" y1="-2.413" x2="2.413" y2="-1.1359" width="0.1524" layer="21" curve="39.5833"/>
<wire x1="1.1359" y1="2.413" x2="2.413" y2="1.1359" width="0.1524" layer="21" curve="-39.5833"/>
<pad name="1" x="0" y="-2.54" drill="0.8128" shape="long"/>
<pad name="3" x="0" y="2.54" drill="0.8128" shape="long"/>
<pad name="2" x="2.54" y="0" drill="0.8128" shape="long"/>
<text x="-1.524" y="3.556" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-4.953" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="TOP3" urn="urn:adsk.eagle:footprint:28457/1">
<description>&lt;b&gt;TOP 3 horizontal&lt;/b&gt;</description>
<wire x1="-7.874" y1="-1.778" x2="7.874" y2="-1.778" width="0.127" layer="21"/>
<wire x1="7.874" y1="8.763" x2="7.874" y2="-1.778" width="0.127" layer="21"/>
<wire x1="-7.874" y1="-1.778" x2="-7.874" y2="8.763" width="0.127" layer="21"/>
<wire x1="-7.874" y1="8.763" x2="-7.874" y2="11.811" width="0.127" layer="21"/>
<wire x1="-7.874" y1="11.811" x2="-3.1242" y2="15.8496" width="0.127" layer="21"/>
<wire x1="7.874" y1="8.763" x2="-7.874" y2="8.763" width="0.127" layer="21"/>
<wire x1="7.874" y1="8.763" x2="7.874" y2="11.811" width="0.127" layer="21"/>
<wire x1="-7.366" y1="-1.27" x2="7.366" y2="-1.27" width="0.0508" layer="21"/>
<wire x1="7.366" y1="8.255" x2="7.366" y2="-1.27" width="0.0508" layer="21"/>
<wire x1="7.366" y1="8.255" x2="-7.366" y2="8.255" width="0.0508" layer="21"/>
<wire x1="-7.366" y1="-1.27" x2="-7.366" y2="8.255" width="0.0508" layer="21"/>
<wire x1="0" y1="17.145" x2="3.1431" y2="15.8431" width="0.127" layer="21" curve="-45.0001"/>
<wire x1="-3.1431" y1="15.8431" x2="0" y2="17.145" width="0.127" layer="21" curve="-45.0001"/>
<wire x1="7.874" y1="11.811" x2="3.1242" y2="15.875" width="0.127" layer="21"/>
<circle x="0" y="12.7" radius="2.159" width="0.127" layer="21"/>
<circle x="0" y="12.7" radius="5.08" width="0" layer="42"/>
<circle x="0" y="12.7" radius="5.08" width="0" layer="43"/>
<pad name="1" x="-5.588" y="-7.62" drill="1.397" shape="long" rot="R90"/>
<pad name="2" x="0" y="-7.62" drill="1.397" shape="long" rot="R90"/>
<pad name="3" x="5.588" y="-7.62" drill="1.397" shape="long" rot="R90"/>
<text x="-5.08" y="5.08" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
<text x="-7.366" y="9.144" size="0.9906" layer="21" ratio="12">A20,3mm</text>
<text x="-5.715" y="-0.635" size="1.27" layer="51" ratio="10">1</text>
<text x="-0.635" y="-0.635" size="1.27" layer="51" ratio="10">2</text>
<text x="4.445" y="-0.635" size="1.27" layer="51" ratio="10">3</text>
<rectangle x1="-6.223" y1="-3.81" x2="-3.81" y2="-1.778" layer="21"/>
<rectangle x1="-1.397" y1="-3.81" x2="1.397" y2="-1.778" layer="21"/>
<rectangle x1="3.81" y1="-3.81" x2="6.223" y2="-1.778" layer="21"/>
<rectangle x1="-6.223" y1="-5.715" x2="-4.953" y2="-3.81" layer="21"/>
<rectangle x1="-6.223" y1="-7.62" x2="-4.953" y2="-5.715" layer="51"/>
<rectangle x1="-0.635" y1="-5.715" x2="0.635" y2="-3.81" layer="21"/>
<rectangle x1="-0.635" y1="-7.62" x2="0.635" y2="-5.715" layer="51"/>
<rectangle x1="4.953" y1="-5.715" x2="6.223" y2="-3.81" layer="21"/>
<rectangle x1="4.953" y1="-7.62" x2="6.223" y2="-5.715" layer="51"/>
<hole x="0" y="12.7" drill="4.1148"/>
</package>
<package name="TOP3V" urn="urn:adsk.eagle:footprint:28684/1">
<description>&lt;b&gt;TOP 3 vertical&lt;/b&gt;</description>
<wire x1="6.985" y1="-5.08" x2="7.366" y2="-4.699" width="0.127" layer="21"/>
<wire x1="7.366" y1="-4.699" x2="7.493" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-6.985" y1="-5.08" x2="-7.366" y2="-4.699" width="0.127" layer="21"/>
<wire x1="-7.366" y1="-4.699" x2="-7.493" y2="-1.524" width="0.127" layer="21"/>
<wire x1="6.985" y1="-5.08" x2="6.35" y2="-5.08" width="0.127" layer="21"/>
<wire x1="6.35" y1="-5.08" x2="4.572" y2="-5.08" width="0.127" layer="51"/>
<wire x1="4.572" y1="-5.08" x2="0.889" y2="-5.08" width="0.127" layer="21"/>
<wire x1="0.889" y1="-5.08" x2="-0.889" y2="-5.08" width="0.127" layer="51"/>
<wire x1="-0.889" y1="-5.08" x2="-4.572" y2="-5.08" width="0.127" layer="21"/>
<wire x1="-4.572" y1="-5.08" x2="-6.35" y2="-5.08" width="0.127" layer="51"/>
<wire x1="-6.35" y1="-5.08" x2="-6.985" y2="-5.08" width="0.127" layer="21"/>
<pad name="B" x="-5.461" y="-3.175" drill="1.397" shape="long" rot="R90"/>
<pad name="C" x="0" y="-3.175" drill="1.397" shape="long" rot="R90"/>
<pad name="E" x="5.461" y="-3.175" drill="1.397" shape="long" rot="R90"/>
<text x="-7.62" y="-6.858" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-6.858" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-6.096" y="-4.445" size="1.27" layer="51" ratio="10">B</text>
<text x="-0.635" y="-4.445" size="1.27" layer="51" ratio="10">C</text>
<text x="4.826" y="-4.445" size="1.27" layer="51" ratio="10">E</text>
<rectangle x1="-7.874" y1="-1.143" x2="7.874" y2="0" layer="21"/>
<rectangle x1="-7.874" y1="-1.524" x2="-6.858" y2="-1.143" layer="21"/>
<rectangle x1="-4.064" y1="-1.524" x2="-1.397" y2="-1.143" layer="21"/>
<rectangle x1="1.397" y1="-1.524" x2="4.064" y2="-1.143" layer="21"/>
<rectangle x1="6.858" y1="-1.524" x2="7.874" y2="-1.143" layer="21"/>
<rectangle x1="-6.858" y1="-1.524" x2="-4.064" y2="-1.143" layer="51"/>
<rectangle x1="-1.397" y1="-1.524" x2="1.397" y2="-1.143" layer="51"/>
<rectangle x1="4.064" y1="-1.524" x2="6.858" y2="-1.143" layer="51"/>
</package>
<package name="SOT23-BEC" urn="urn:adsk.eagle:footprint:28685/1">
<description>TO-236 ITT Intermetall</description>
<wire x1="1.4224" y1="0.6604" x2="1.4224" y2="-0.6604" width="0.127" layer="51"/>
<wire x1="1.4224" y1="-0.6604" x2="-1.4224" y2="-0.6604" width="0.127" layer="51"/>
<wire x1="-1.4224" y1="-0.6604" x2="-1.4224" y2="0.6604" width="0.127" layer="51"/>
<wire x1="-1.4224" y1="0.6604" x2="1.4224" y2="0.6604" width="0.127" layer="51"/>
<smd name="C" x="0" y="1.1" dx="1" dy="1.4" layer="1"/>
<smd name="E" x="0.95" y="-1.1" dx="1" dy="1.4" layer="1"/>
<smd name="B" x="-0.95" y="-1.1" dx="1" dy="1.4" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.2286" y1="0.7112" x2="0.2286" y2="1.2954" layer="51"/>
<rectangle x1="0.7112" y1="-1.2954" x2="1.1684" y2="-0.7112" layer="51"/>
<rectangle x1="-1.1684" y1="-1.2954" x2="-0.7112" y2="-0.7112" layer="51"/>
</package>
<package name="SOT23-EBC" urn="urn:adsk.eagle:footprint:28686/1">
<description>&lt;b&gt;SOT-23 (EBC)&lt;/b&gt;</description>
<wire x1="1.4224" y1="0.6604" x2="1.4224" y2="-0.6604" width="0.127" layer="51"/>
<wire x1="1.4224" y1="-0.6604" x2="-1.4224" y2="-0.6604" width="0.127" layer="51"/>
<wire x1="-1.4224" y1="-0.6604" x2="-1.4224" y2="0.6604" width="0.127" layer="51"/>
<wire x1="-1.4224" y1="0.6604" x2="1.4224" y2="0.6604" width="0.127" layer="51"/>
<smd name="C" x="0" y="1.1" dx="1" dy="1.4" layer="1"/>
<smd name="B" x="0.95" y="-1.1" dx="1" dy="1.4" layer="1"/>
<smd name="E" x="-0.95" y="-1.1" dx="1" dy="1.4" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.2286" y1="0.7112" x2="0.2286" y2="1.2954" layer="51"/>
<rectangle x1="0.7112" y1="-1.2954" x2="1.1684" y2="-0.7112" layer="51"/>
<rectangle x1="-1.1684" y1="-1.2954" x2="-0.7112" y2="-0.7112" layer="51"/>
</package>
<package name="TO92-BCE" urn="urn:adsk.eagle:footprint:29858/1">
<description>&lt;b&gt;TO-92&lt;/b&gt; Pads In Line B C E from top&lt;p&gt;</description>
<wire x1="-2.095" y1="-1.651" x2="2.095" y2="-1.651" width="0.127" layer="21"/>
<wire x1="-2.413" y1="1.1359" x2="2.413" y2="1.1359" width="0.127" layer="21" curve="-129.583"/>
<wire x1="1.136" y1="-0.127" x2="-1.136" y2="-0.127" width="0.127" layer="51"/>
<wire x1="-2.413" y1="1.1359" x2="-2.664" y2="-0.127" width="0.127" layer="51" curve="27.9376"/>
<wire x1="-2.664" y1="-0.127" x2="-2.413" y2="-1.1359" width="0.127" layer="51" curve="22.4788"/>
<wire x1="-1.404" y1="-0.127" x2="-2.664" y2="-0.127" width="0.127" layer="51"/>
<wire x1="-2.4135" y1="-1.1359" x2="-2.095" y2="-1.651" width="0.127" layer="21" curve="13.0385"/>
<wire x1="-1.136" y1="-0.127" x2="-1.404" y2="-0.127" width="0.127" layer="21"/>
<wire x1="2.413" y1="-1.1359" x2="2.664" y2="-0.127" width="0.127" layer="51" curve="22.4788"/>
<wire x1="2.664" y1="-0.127" x2="2.413" y2="1.1359" width="0.127" layer="51" curve="27.9376"/>
<wire x1="2.664" y1="-0.127" x2="1.404" y2="-0.127" width="0.127" layer="51"/>
<wire x1="1.404" y1="-0.127" x2="1.136" y2="-0.127" width="0.127" layer="21"/>
<wire x1="2.095" y1="-1.651" x2="2.4247" y2="-1.1118" width="0.127" layer="21" curve="13.6094"/>
<pad name="C" x="0" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="E" x="2.54" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="B" x="-2.54" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="-2.54" y="3.048" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-3.302" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="TO92-CBE" urn="urn:adsk.eagle:footprint:29859/1">
<description>&lt;b&gt;TO-92&lt;/b&gt; Pads In Line  C B E from top&lt;p&gt;</description>
<wire x1="-2.095" y1="-1.651" x2="2.095" y2="-1.651" width="0.127" layer="21"/>
<wire x1="-2.413" y1="1.1359" x2="2.413" y2="1.1359" width="0.127" layer="21" curve="-129.583"/>
<wire x1="1.136" y1="-0.127" x2="-1.136" y2="-0.127" width="0.127" layer="51"/>
<wire x1="-2.413" y1="1.1359" x2="-2.664" y2="-0.127" width="0.127" layer="51" curve="27.9376"/>
<wire x1="-2.664" y1="-0.127" x2="-2.413" y2="-1.1359" width="0.127" layer="51" curve="22.4788"/>
<wire x1="-1.404" y1="-0.127" x2="-2.664" y2="-0.127" width="0.127" layer="51"/>
<wire x1="-2.4135" y1="-1.1359" x2="-2.095" y2="-1.651" width="0.127" layer="21" curve="13.0385"/>
<wire x1="-1.136" y1="-0.127" x2="-1.404" y2="-0.127" width="0.127" layer="21"/>
<wire x1="2.413" y1="-1.1359" x2="2.664" y2="-0.127" width="0.127" layer="51" curve="22.4788"/>
<wire x1="2.664" y1="-0.127" x2="2.413" y2="1.1359" width="0.127" layer="51" curve="27.9376"/>
<wire x1="2.664" y1="-0.127" x2="1.404" y2="-0.127" width="0.127" layer="51"/>
<wire x1="1.404" y1="-0.127" x2="1.136" y2="-0.127" width="0.127" layer="21"/>
<wire x1="2.095" y1="-1.651" x2="2.4247" y2="-1.1118" width="0.127" layer="21" curve="13.6094"/>
<pad name="C" x="-2.54" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="E" x="2.54" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="B" x="0" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="-2.54" y="3.048" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-3.302" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="TO92-BEC" urn="urn:adsk.eagle:footprint:29860/1">
<description>&lt;b&gt;TO-92&lt;/b&gt; Pads In Line  B E C from top&lt;p&gt;</description>
<wire x1="-2.095" y1="-1.651" x2="2.095" y2="-1.651" width="0.127" layer="21"/>
<wire x1="-2.413" y1="1.1359" x2="2.413" y2="1.1359" width="0.127" layer="21" curve="-129.583"/>
<wire x1="1.136" y1="-0.127" x2="-1.136" y2="-0.127" width="0.127" layer="51"/>
<wire x1="-2.413" y1="1.1359" x2="-2.664" y2="-0.127" width="0.127" layer="51" curve="27.9376"/>
<wire x1="-2.664" y1="-0.127" x2="-2.413" y2="-1.1359" width="0.127" layer="51" curve="22.4788"/>
<wire x1="-1.404" y1="-0.127" x2="-2.664" y2="-0.127" width="0.127" layer="51"/>
<wire x1="-2.4135" y1="-1.1359" x2="-2.095" y2="-1.651" width="0.127" layer="21" curve="13.0385"/>
<wire x1="-1.136" y1="-0.127" x2="-1.404" y2="-0.127" width="0.127" layer="21"/>
<wire x1="2.413" y1="-1.1359" x2="2.664" y2="-0.127" width="0.127" layer="51" curve="22.4788"/>
<wire x1="2.664" y1="-0.127" x2="2.413" y2="1.1359" width="0.127" layer="51" curve="27.9376"/>
<wire x1="2.664" y1="-0.127" x2="1.404" y2="-0.127" width="0.127" layer="51"/>
<wire x1="1.404" y1="-0.127" x2="1.136" y2="-0.127" width="0.127" layer="21"/>
<wire x1="2.095" y1="-1.651" x2="2.4247" y2="-1.1118" width="0.127" layer="21" curve="13.6094"/>
<pad name="C" x="2.54" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="E" x="0" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="B" x="-2.54" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="-2.54" y="3.048" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-3.302" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="TO92-CEB" urn="urn:adsk.eagle:footprint:29861/1">
<description>&lt;b&gt;TO-92&lt;/b&gt; Pads In Line C E B from top&lt;p&gt;</description>
<wire x1="-2.095" y1="-1.651" x2="2.095" y2="-1.651" width="0.127" layer="21"/>
<wire x1="-2.413" y1="1.1359" x2="2.413" y2="1.1359" width="0.127" layer="21" curve="-129.583"/>
<wire x1="1.136" y1="-0.127" x2="-1.136" y2="-0.127" width="0.127" layer="51"/>
<wire x1="-2.413" y1="1.1359" x2="-2.664" y2="-0.127" width="0.127" layer="51" curve="27.9376"/>
<wire x1="-2.664" y1="-0.127" x2="-2.413" y2="-1.1359" width="0.127" layer="51" curve="22.4788"/>
<wire x1="-1.404" y1="-0.127" x2="-2.664" y2="-0.127" width="0.127" layer="51"/>
<wire x1="-2.4135" y1="-1.1359" x2="-2.095" y2="-1.651" width="0.127" layer="21" curve="13.0385"/>
<wire x1="-1.136" y1="-0.127" x2="-1.404" y2="-0.127" width="0.127" layer="21"/>
<wire x1="2.413" y1="-1.1359" x2="2.664" y2="-0.127" width="0.127" layer="51" curve="22.4788"/>
<wire x1="2.664" y1="-0.127" x2="2.413" y2="1.1359" width="0.127" layer="51" curve="27.9376"/>
<wire x1="2.664" y1="-0.127" x2="1.404" y2="-0.127" width="0.127" layer="51"/>
<wire x1="1.404" y1="-0.127" x2="1.136" y2="-0.127" width="0.127" layer="21"/>
<wire x1="2.095" y1="-1.651" x2="2.4247" y2="-1.1118" width="0.127" layer="21" curve="13.6094"/>
<pad name="C" x="-2.54" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="E" x="0" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="B" x="2.54" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="-2.54" y="3.048" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-3.302" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="SC59-BEC" urn="urn:adsk.eagle:footprint:28691/1">
<description>SC59 (SOT23) Motorola</description>
<wire x1="1.4224" y1="0.8604" x2="1.4224" y2="-0.8604" width="0.127" layer="51"/>
<wire x1="1.4224" y1="-0.8604" x2="-1.4224" y2="-0.8604" width="0.127" layer="51"/>
<wire x1="-1.4224" y1="-0.8604" x2="-1.4224" y2="0.8604" width="0.127" layer="51"/>
<wire x1="-1.4224" y1="0.8604" x2="1.4224" y2="0.8604" width="0.127" layer="51"/>
<smd name="C" x="0" y="1.2" dx="1" dy="1.4" layer="1"/>
<smd name="E" x="0.95" y="-1.2" dx="1" dy="1.4" layer="1"/>
<smd name="B" x="-0.95" y="-1.2" dx="1" dy="1.4" layer="1"/>
<text x="-1.905" y="1.905" size="1.27" layer="25">&gt;NAME</text>
<text x="-1.905" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.2286" y1="0.9112" x2="0.2286" y2="1.4954" layer="51"/>
<rectangle x1="0.7112" y1="-1.4954" x2="1.1684" y2="-0.9112" layer="51"/>
<rectangle x1="-1.1684" y1="-1.4954" x2="-0.7112" y2="-0.9112" layer="51"/>
</package>
<package name="SOT89-BCE" urn="urn:adsk.eagle:footprint:29863/1">
<description>SOT89 Basis Collector Emitter</description>
<wire x1="2.235" y1="-1.245" x2="-2.235" y2="-1.245" width="0.127" layer="51"/>
<wire x1="2.235" y1="1.219" x2="2.235" y2="-1.245" width="0.127" layer="51"/>
<wire x1="-2.235" y1="-1.245" x2="-2.235" y2="1.219" width="0.127" layer="51"/>
<wire x1="-2.235" y1="1.219" x2="2.235" y2="1.219" width="0.127" layer="51"/>
<wire x1="-0.7874" y1="1.5748" x2="-0.3556" y2="2.0066" width="0.1998" layer="51"/>
<wire x1="-0.3556" y1="2.0066" x2="0.3556" y2="2.0066" width="0.1998" layer="51"/>
<wire x1="0.3556" y1="2.0066" x2="0.7874" y2="1.5748" width="0.1998" layer="51"/>
<wire x1="0.7874" y1="1.5748" x2="0.7874" y2="1.2954" width="0.1998" layer="51"/>
<wire x1="0.7874" y1="1.2954" x2="-0.7874" y2="1.2954" width="0.1998" layer="51"/>
<wire x1="-0.7874" y1="1.2954" x2="-0.7874" y2="1.5748" width="0.1998" layer="51"/>
<smd name="B" x="-1.499" y="-1.981" dx="0.8" dy="1.4" layer="1"/>
<smd name="E" x="1.499" y="-1.981" dx="0.8" dy="1.4" layer="1"/>
<smd name="C" x="0" y="-1.727" dx="0.8" dy="1.9" layer="1" stop="no" cream="no"/>
<text x="-2.54" y="3.175" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.4051" y="-4.3449" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-1.7272" y1="-2.1082" x2="-1.27" y2="-1.27" layer="51"/>
<rectangle x1="1.27" y1="-2.1082" x2="1.7272" y2="-1.27" layer="51"/>
<rectangle x1="-0.2794" y1="-2.1082" x2="0.2794" y2="-1.27" layer="51"/>
<rectangle x1="-0.4" y1="-2.68" x2="0.4" y2="-1.28" layer="31"/>
<rectangle x1="-0.5" y1="-2.78" x2="0.5" y2="-1.18" layer="29"/>
<polygon width="0.1998" layer="51">
<vertex x="-0.7874" y="1.3208"/>
<vertex x="-0.7874" y="1.5748"/>
<vertex x="-0.3556" y="2.0066"/>
<vertex x="0.3048" y="2.0066"/>
<vertex x="0.3556" y="2.0066"/>
<vertex x="0.7874" y="1.5748"/>
<vertex x="0.7874" y="1.2954"/>
<vertex x="-0.7874" y="1.2954"/>
</polygon>
<polygon width="1.7" layer="29">
<vertex x="-0.275" y="2.025"/>
<vertex x="-0.275" y="-0.15"/>
<vertex x="0.25" y="-0.15"/>
<vertex x="0.25" y="2.025"/>
</polygon>
<polygon width="1.3" layer="31">
<vertex x="-0.25" y="2.05"/>
<vertex x="0.275" y="2.05"/>
<vertex x="0.275" y="-0.125"/>
<vertex x="-0.25" y="-0.125"/>
</polygon>
<polygon width="0.3" layer="1">
<vertex x="0.25" y="-2.05"/>
<vertex x="-0.25" y="-2.05"/>
<vertex x="-0.25" y="-0.75" curve="-90"/>
<vertex x="-0.875" y="-0.125"/>
<vertex x="-0.875" y="2.05" curve="-90"/>
<vertex x="-0.225" y="2.625"/>
<vertex x="0.25" y="2.625" curve="-90"/>
<vertex x="0.875" y="2.025"/>
<vertex x="0.875" y="-0.125" curve="-90"/>
<vertex x="0.25" y="-0.75"/>
</polygon>
</package>
<package name="SOT37" urn="urn:adsk.eagle:footprint:28693/1">
<description>&lt;b&gt;SOT-37&lt;/b&gt;&lt;p&gt;
Motorola CASE 317A-01 ISSUE B&lt;br&gt;
http://www.ee.siue.edu/~alozows/library/datasheets/BFR96.pdf</description>
<wire x1="5.3086" y1="0.508" x2="5.3086" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="-5.3086" x2="0.508" y2="-5.3086" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="6.5024" x2="0.508" y2="6.5024" width="0.1524" layer="51"/>
<wire x1="0.508" y1="-2.2352" x2="0.508" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-2.794" x2="0.508" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-3.048" x2="0.635" y2="-3.429" width="0.1524" layer="51"/>
<wire x1="0.508" y1="-3.429" x2="0.635" y2="-3.429" width="0.1524" layer="51"/>
<wire x1="0.508" y1="-3.429" x2="0.508" y2="-5.3086" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="-2.2352" x2="-0.508" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-2.794" x2="-0.508" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-3.048" x2="-0.635" y2="-3.429" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="-3.429" x2="-0.635" y2="-3.429" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="-3.429" x2="-0.508" y2="-5.3086" width="0.1524" layer="51"/>
<wire x1="2.2352" y1="-0.508" x2="2.794" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="2.794" y1="-0.635" x2="2.794" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="3.048" y1="-0.635" x2="3.429" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="3.429" y1="-0.508" x2="3.429" y2="-0.635" width="0.1524" layer="51"/>
<wire x1="3.429" y1="-0.508" x2="5.3086" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="2.2098" y1="0.508" x2="2.794" y2="0.508" width="0.1524" layer="21"/>
<wire x1="2.794" y1="0.635" x2="2.794" y2="0.508" width="0.1524" layer="21"/>
<wire x1="3.048" y1="0.635" x2="3.429" y2="0.635" width="0.1524" layer="51"/>
<wire x1="3.429" y1="0.508" x2="3.429" y2="0.635" width="0.1524" layer="51"/>
<wire x1="3.429" y1="0.508" x2="5.3086" y2="0.508" width="0.1524" layer="51"/>
<wire x1="0.508" y1="2.2352" x2="0.508" y2="2.794" width="0.1524" layer="21"/>
<wire x1="0.635" y1="2.794" x2="0.508" y2="2.794" width="0.1524" layer="21"/>
<wire x1="0.635" y1="3.048" x2="0.635" y2="3.429" width="0.1524" layer="51"/>
<wire x1="0.508" y1="3.429" x2="0.635" y2="3.429" width="0.1524" layer="51"/>
<wire x1="0.508" y1="3.429" x2="0.508" y2="6.5024" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="2.2352" x2="-0.508" y2="2.794" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="2.794" x2="-0.508" y2="2.794" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="3.048" x2="-0.635" y2="3.429" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="3.429" x2="-0.635" y2="3.429" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="3.429" x2="-0.508" y2="6.5024" width="0.1524" layer="51"/>
<wire x1="-0.635" y1="-2.794" x2="-0.635" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-2.794" x2="0.635" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="3.048" y1="-0.635" x2="2.794" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="3.048" y1="0.635" x2="2.794" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0.635" y1="2.794" x2="0.635" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="2.794" x2="-0.635" y2="3.048" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="2.286" width="0.1524" layer="21"/>
<circle x="0" y="1.524" radius="0.508" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="3.175" width="0" layer="41"/>
<circle x="0" y="0" radius="3.175" width="0" layer="42"/>
<smd name="3" x="0" y="-5.08" dx="1.9304" dy="3.81" layer="1"/>
<smd name="2" x="5.08" y="0" dx="3.81" dy="1.9304" layer="1"/>
<smd name="1" x="0" y="5.08" dx="1.9304" dy="3.81" layer="1"/>
<text x="4.445" y="1.27" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="6.35" y="1.27" size="1.27" layer="27" ratio="10" rot="R90">&gt;VALUE</text>
<text x="3.175" y="-2.286" size="1.016" layer="21" ratio="10">E</text>
<text x="1.27" y="-4.191" size="1.016" layer="21" ratio="10">B</text>
<text x="1.27" y="3.175" size="1.016" layer="21" ratio="10">C</text>
<text x="-1.524" y="-4.318" size="1.27" layer="47">5,1</text>
<rectangle x1="-0.508" y1="-2.794" x2="0.508" y2="-2.413" layer="21"/>
<rectangle x1="-0.635" y1="-3.048" x2="0.635" y2="-2.794" layer="21"/>
<rectangle x1="-0.635" y1="-3.429" x2="0.635" y2="-3.048" layer="51"/>
<rectangle x1="-0.508" y1="-5.334" x2="0.508" y2="-3.429" layer="51"/>
<rectangle x1="2.413" y1="-0.508" x2="2.794" y2="0.508" layer="21"/>
<rectangle x1="2.794" y1="-0.635" x2="3.048" y2="0.635" layer="21"/>
<rectangle x1="3.048" y1="-0.635" x2="3.429" y2="0.635" layer="51"/>
<rectangle x1="3.429" y1="-0.508" x2="5.334" y2="0.508" layer="51"/>
<rectangle x1="-0.508" y1="3.429" x2="0.508" y2="6.477" layer="51"/>
<rectangle x1="-0.635" y1="3.048" x2="0.635" y2="3.429" layer="51"/>
<rectangle x1="-0.635" y1="2.794" x2="0.635" y2="3.048" layer="21"/>
<rectangle x1="-0.508" y1="2.413" x2="0.508" y2="2.794" layer="21"/>
<hole x="0" y="0" drill="5.08"/>
</package>
<package name="TO92-ECB" urn="urn:adsk.eagle:footprint:29865/1">
<description>&lt;b&gt;TO-92&lt;/b&gt; Pads In Line E C B from top&lt;p&gt;</description>
<wire x1="-2.095" y1="-1.651" x2="2.095" y2="-1.651" width="0.127" layer="21"/>
<wire x1="-2.413" y1="1.1359" x2="2.413" y2="1.1359" width="0.127" layer="21" curve="-129.583"/>
<wire x1="1.136" y1="-0.127" x2="-1.136" y2="-0.127" width="0.127" layer="51"/>
<wire x1="-2.413" y1="1.1359" x2="-2.664" y2="-0.127" width="0.127" layer="51" curve="27.9407"/>
<wire x1="-2.664" y1="-0.127" x2="-2.413" y2="-1.1359" width="0.127" layer="51" curve="22.4788"/>
<wire x1="-1.404" y1="-0.127" x2="-2.664" y2="-0.127" width="0.127" layer="51"/>
<wire x1="-2.4135" y1="-1.1359" x2="-2.095" y2="-1.651" width="0.127" layer="21" curve="13.0385"/>
<wire x1="-1.136" y1="-0.127" x2="-1.404" y2="-0.127" width="0.127" layer="21"/>
<wire x1="2.413" y1="-1.1359" x2="2.664" y2="-0.127" width="0.127" layer="51" curve="22.4788"/>
<wire x1="2.664" y1="-0.127" x2="2.413" y2="1.1359" width="0.127" layer="51" curve="27.9333"/>
<wire x1="2.664" y1="-0.127" x2="1.404" y2="-0.127" width="0.127" layer="51"/>
<wire x1="1.404" y1="-0.127" x2="1.136" y2="-0.127" width="0.127" layer="21"/>
<wire x1="2.095" y1="-1.651" x2="2.4247" y2="-1.1118" width="0.127" layer="21" curve="13.6094"/>
<pad name="C" x="0" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="E" x="-2.54" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="B" x="2.54" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="-2.54" y="3.048" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-3.302" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="TO92-EBC" urn="urn:adsk.eagle:package:29915/2" locally_modified="yes" type="model">
<description>TO-92 Pads In Line E B C from top</description>
<packageinstances>
<packageinstance name="TO92-EBC"/>
</packageinstances>
</package3d>
<package3d name="TO3" urn="urn:adsk.eagle:package:29899/2" type="model">
<description>TO-3</description>
<packageinstances>
<packageinstance name="TO3"/>
</packageinstances>
</package3d>
<package3d name="TO18" urn="urn:adsk.eagle:package:28526/2" type="model">
<description>TO 18</description>
<packageinstances>
<packageinstance name="TO18"/>
</packageinstances>
</package3d>
<package3d name="TO18-" urn="urn:adsk.eagle:package:28983/2" type="model">
<description>TO-18</description>
<packageinstances>
<packageinstance name="TO18-"/>
</packageinstances>
</package3d>
<package3d name="TO202" urn="urn:adsk.eagle:package:28525/1" type="box">
<description>TO 202 horizontal</description>
<packageinstances>
<packageinstance name="TO202"/>
</packageinstances>
</package3d>
<package3d name="TO202V" urn="urn:adsk.eagle:package:28741/1" type="box">
<description>TO 202 vertical</description>
<packageinstances>
<packageinstance name="TO202V"/>
</packageinstances>
</package3d>
<package3d name="SOT93" urn="urn:adsk.eagle:package:29486/1" type="box">
<description>SOT-93
grid 5.45 mm</description>
<packageinstances>
<packageinstance name="SOT93"/>
</packageinstances>
</package3d>
<package3d name="SOT93V" urn="urn:adsk.eagle:package:28764/1" type="box">
<description>SOT-93
grid 5.45 mm</description>
<packageinstances>
<packageinstance name="SOT93V"/>
</packageinstances>
</package3d>
<package3d name="TO126" urn="urn:adsk.eagle:package:29898/1" type="box">
<description>TO 126 horizontal</description>
<packageinstances>
<packageinstance name="TO126"/>
</packageinstances>
</package3d>
<package3d name="TO126V" urn="urn:adsk.eagle:package:28743/1" type="box">
<description>TO 126 vertical</description>
<packageinstances>
<packageinstance name="TO126V"/>
</packageinstances>
</package3d>
<package3d name="TO218" urn="urn:adsk.eagle:package:28522/1" type="box">
<description>TO 218 horizontal</description>
<packageinstances>
<packageinstance name="TO218"/>
</packageinstances>
</package3d>
<package3d name="TO218V" urn="urn:adsk.eagle:package:28519/1" type="box">
<description>TO 218 vertical</description>
<packageinstances>
<packageinstance name="TO218V"/>
</packageinstances>
</package3d>
<package3d name="TO220" urn="urn:adsk.eagle:package:28520/1" type="box">
<description>TO 220 horizontal</description>
<packageinstances>
<packageinstance name="TO220"/>
</packageinstances>
</package3d>
<package3d name="TO220V" urn="urn:adsk.eagle:package:28746/2" type="model">
<description>TO 220 vertical</description>
<packageinstances>
<packageinstance name="TO220V"/>
</packageinstances>
</package3d>
<package3d name="TO225AA" urn="urn:adsk.eagle:package:29905/1" type="box">
<description>TO-225
grid 2.54 mm, vertical</description>
<packageinstances>
<packageinstance name="TO225AA"/>
</packageinstances>
</package3d>
<package3d name="TO39" urn="urn:adsk.eagle:package:28730/1" type="box">
<description>TO-39</description>
<packageinstances>
<packageinstance name="TO39"/>
</packageinstances>
</package3d>
<package3d name="TO5" urn="urn:adsk.eagle:package:28729/1" type="box">
<description>TO 5</description>
<packageinstances>
<packageinstance name="TO5"/>
</packageinstances>
</package3d>
<package3d name="TO66" urn="urn:adsk.eagle:package:29220/1" type="box">
<description>TO-66</description>
<packageinstances>
<packageinstance name="TO66"/>
</packageinstances>
</package3d>
<package3d name="TO92" urn="urn:adsk.eagle:package:28523/2" type="model">
<description>TO 92</description>
<packageinstances>
<packageinstance name="TO92"/>
</packageinstances>
</package3d>
<package3d name="TO92-E1" urn="urn:adsk.eagle:package:29916/2" type="model">
<description>TO-92 Pads Triangle Reverse</description>
<packageinstances>
<packageinstance name="TO92-E1"/>
</packageinstances>
</package3d>
<package3d name="TO92L" urn="urn:adsk.eagle:package:29918/1" type="box">
<description>TO-92
grid 5.08 mm</description>
<packageinstances>
<packageinstance name="TO92L"/>
</packageinstances>
</package3d>
<package3d name="TOP3" urn="urn:adsk.eagle:package:28521/1" type="box">
<description>TOP 3 horizontal</description>
<packageinstances>
<packageinstance name="TOP3"/>
</packageinstances>
</package3d>
<package3d name="TOP3V" urn="urn:adsk.eagle:package:28768/1" type="box">
<description>TOP 3 vertical</description>
<packageinstances>
<packageinstance name="TOP3V"/>
</packageinstances>
</package3d>
<package3d name="SOT23-BEC" urn="urn:adsk.eagle:package:28748/2" type="model">
<description>TO-236 ITT Intermetall</description>
<packageinstances>
<packageinstance name="SOT23-BEC"/>
</packageinstances>
</package3d>
<package3d name="SOT23-EBC" urn="urn:adsk.eagle:package:28750/2" type="model">
<description>SOT-23 (EBC)</description>
<packageinstances>
<packageinstance name="SOT23-EBC"/>
</packageinstances>
</package3d>
<package3d name="TO92-BCE" urn="urn:adsk.eagle:package:29924/2" type="model">
<description>TO-92 Pads In Line B C E from top</description>
<packageinstances>
<packageinstance name="TO92-BCE"/>
</packageinstances>
</package3d>
<package3d name="TO92-CBE" urn="urn:adsk.eagle:package:29921/2" type="model">
<description>TO-92 Pads In Line  C B E from top</description>
<packageinstances>
<packageinstance name="TO92-CBE"/>
</packageinstances>
</package3d>
<package3d name="TO92-BEC" urn="urn:adsk.eagle:package:29922/2" type="model">
<description>TO-92 Pads In Line  B E C from top</description>
<packageinstances>
<packageinstance name="TO92-BEC"/>
</packageinstances>
</package3d>
<package3d name="TO92-CEB" urn="urn:adsk.eagle:package:29923/2" type="model">
<description>TO-92 Pads In Line C E B from top</description>
<packageinstances>
<packageinstance name="TO92-CEB"/>
</packageinstances>
</package3d>
<package3d name="SC59-BEC" urn="urn:adsk.eagle:package:28754/2" type="model">
<description>SC59 (SOT23) Motorola</description>
<packageinstances>
<packageinstance name="SC59-BEC"/>
</packageinstances>
</package3d>
<package3d name="SOT89-BCE" urn="urn:adsk.eagle:package:29925/3" type="model">
<description>SOT89 Basis Collector Emitter</description>
<packageinstances>
<packageinstance name="SOT89-BCE"/>
</packageinstances>
</package3d>
<package3d name="SOT37" urn="urn:adsk.eagle:package:28760/1" type="box">
<description>SOT-37
Motorola CASE 317A-01 ISSUE B
http://www.ee.siue.edu/~alozows/library/datasheets/BFR96.pdf</description>
<packageinstances>
<packageinstance name="SOT37"/>
</packageinstances>
</package3d>
<package3d name="TO92-ECB" urn="urn:adsk.eagle:package:29929/2" type="model">
<description>TO-92 Pads In Line E C B from top</description>
<packageinstances>
<packageinstance name="TO92-ECB"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="NPN" urn="urn:adsk.eagle:symbol:29821/1">
<wire x1="2.54" y1="2.54" x2="0.508" y2="1.524" width="0.1524" layer="94"/>
<wire x1="1.778" y1="-1.524" x2="2.54" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="2.54" y1="-2.54" x2="1.27" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="1.778" y2="-1.524" width="0.1524" layer="94"/>
<wire x1="1.54" y1="-2.04" x2="0.308" y2="-1.424" width="0.1524" layer="94"/>
<wire x1="1.524" y1="-2.413" x2="2.286" y2="-2.413" width="0.254" layer="94"/>
<wire x1="2.286" y1="-2.413" x2="1.778" y2="-1.778" width="0.254" layer="94"/>
<wire x1="1.778" y1="-1.778" x2="1.524" y2="-2.286" width="0.254" layer="94"/>
<wire x1="1.524" y1="-2.286" x2="1.905" y2="-2.286" width="0.254" layer="94"/>
<wire x1="1.905" y1="-2.286" x2="1.778" y2="-2.032" width="0.254" layer="94"/>
<text x="-10.16" y="7.62" size="1.778" layer="95">&gt;NAME</text>
<text x="-10.16" y="5.08" size="1.778" layer="96">&gt;VALUE</text>
<rectangle x1="-0.254" y1="-2.54" x2="0.508" y2="2.54" layer="94"/>
<pin name="B" x="-2.54" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
<pin name="E" x="2.54" y="-5.08" visible="off" length="short" direction="pas" swaplevel="3" rot="R90"/>
<pin name="C" x="2.54" y="5.08" visible="off" length="short" direction="pas" swaplevel="2" rot="R270"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="*-NPN-" urn="urn:adsk.eagle:component:30081/21" locally_modified="yes" prefix="T" uservalue="yes">
<description>NPN Transistror&lt;p&gt;
BF959 corrected 2008.03.06&lt;br&gt;</description>
<gates>
<gate name="G$1" symbol="NPN" x="0" y="0"/>
</gates>
<devices>
<device name="TO3/" package="TO3">
<connects>
<connect gate="G$1" pin="B" pad="B"/>
<connect gate="G$1" pin="C" pad="C"/>
<connect gate="G$1" pin="E" pad="E"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29899/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TO18-EBC" package="TO18">
<connects>
<connect gate="G$1" pin="B" pad="2"/>
<connect gate="G$1" pin="C" pad="3"/>
<connect gate="G$1" pin="E" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28526/2"/>
</package3dinstances>
<technologies>
<technology name="2N2218"/>
<technology name="2N2222"/>
<technology name="2N2222A"/>
<technology name="2N2369"/>
<technology name="2N2369A"/>
<technology name="2N2484"/>
<technology name="2N2896"/>
<technology name="2N3700"/>
<technology name="2N930"/>
<technology name="BC107A"/>
<technology name="BC107B"/>
<technology name="BC108B"/>
<technology name="BC108C"/>
<technology name="BCY58-IX"/>
<technology name="BCY58-VIII"/>
<technology name="BCY59-VII"/>
<technology name="BCY59-VIII"/>
<technology name="BCY59-X"/>
<technology name="BSS71"/>
<technology name="BSS72"/>
<technology name="BSS73"/>
<technology name="BSX20"/>
</technologies>
</device>
<device name="TO18-" package="TO18-">
<connects>
<connect gate="G$1" pin="B" pad="2"/>
<connect gate="G$1" pin="C" pad="3"/>
<connect gate="G$1" pin="E" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28983/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TO202" package="TO202">
<connects>
<connect gate="G$1" pin="B" pad="3"/>
<connect gate="G$1" pin="C" pad="2"/>
<connect gate="G$1" pin="E" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28525/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TO202V" package="TO202V">
<connects>
<connect gate="G$1" pin="B" pad="3"/>
<connect gate="G$1" pin="C" pad="2"/>
<connect gate="G$1" pin="E" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28741/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SOT93" package="SOT93">
<connects>
<connect gate="G$1" pin="B" pad="B"/>
<connect gate="G$1" pin="C" pad="C"/>
<connect gate="G$1" pin="E" pad="E"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29486/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SOT93V" package="SOT93V">
<connects>
<connect gate="G$1" pin="B" pad="B"/>
<connect gate="G$1" pin="C" pad="C"/>
<connect gate="G$1" pin="E" pad="E"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28764/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TO126" package="TO126">
<connects>
<connect gate="G$1" pin="B" pad="3"/>
<connect gate="G$1" pin="C" pad="2"/>
<connect gate="G$1" pin="E" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29898/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TO126V" package="TO126V">
<connects>
<connect gate="G$1" pin="B" pad="3"/>
<connect gate="G$1" pin="C" pad="2"/>
<connect gate="G$1" pin="E" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28743/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TO218" package="TO218">
<connects>
<connect gate="G$1" pin="B" pad="1"/>
<connect gate="G$1" pin="C" pad="2"/>
<connect gate="G$1" pin="E" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28522/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TO218V" package="TO218V">
<connects>
<connect gate="G$1" pin="B" pad="1"/>
<connect gate="G$1" pin="C" pad="2"/>
<connect gate="G$1" pin="E" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28519/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TO220" package="TO220">
<connects>
<connect gate="G$1" pin="B" pad="1"/>
<connect gate="G$1" pin="C" pad="2"/>
<connect gate="G$1" pin="E" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28520/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TO220V" package="TO220V">
<connects>
<connect gate="G$1" pin="B" pad="1"/>
<connect gate="G$1" pin="C" pad="2"/>
<connect gate="G$1" pin="E" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28746/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TO225AA" package="TO225AA">
<connects>
<connect gate="G$1" pin="B" pad="1"/>
<connect gate="G$1" pin="C" pad="2"/>
<connect gate="G$1" pin="E" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29905/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TO39-EBC" package="TO39">
<connects>
<connect gate="G$1" pin="B" pad="2"/>
<connect gate="G$1" pin="C" pad="3"/>
<connect gate="G$1" pin="E" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28730/1"/>
</package3dinstances>
<technologies>
<technology name="2N1613"/>
<technology name="2N1711"/>
<technology name="2N1893"/>
<technology name="2N2102"/>
<technology name="2N2218"/>
<technology name="2N2219"/>
<technology name="2N2219A"/>
<technology name="2N3019"/>
<technology name="2N3020"/>
<technology name="2N3439"/>
<technology name="2N3440"/>
<technology name="BC140-10"/>
<technology name="BC140-16"/>
<technology name="BC141-10"/>
<technology name="BC141-16"/>
<technology name="BFY50"/>
<technology name="BSX45-16"/>
<technology name="BSX46-10"/>
<technology name="BSX46-16"/>
</technologies>
</device>
<device name="TO5" package="TO5">
<connects>
<connect gate="G$1" pin="B" pad="2"/>
<connect gate="G$1" pin="C" pad="3"/>
<connect gate="G$1" pin="E" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28729/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TO66" package="TO66">
<connects>
<connect gate="G$1" pin="B" pad="2"/>
<connect gate="G$1" pin="C" pad="3"/>
<connect gate="G$1" pin="E" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29220/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TO92" package="TO92">
<connects>
<connect gate="G$1" pin="B" pad="2"/>
<connect gate="G$1" pin="C" pad="3"/>
<connect gate="G$1" pin="E" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28523/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TO92-EBC" package="TO92-EBC">
<connects>
<connect gate="G$1" pin="B" pad="B"/>
<connect gate="G$1" pin="C" pad="C"/>
<connect gate="G$1" pin="E" pad="E"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29915/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
<technology name="29"/>
<technology name="92"/>
<technology name="BC317"/>
<technology name="BC318"/>
<technology name="BC319"/>
<technology name="MPSA06"/>
<technology name="MPSA13"/>
<technology name="MPSA14"/>
<technology name="MPSA18"/>
<technology name="MPSA28"/>
<technology name="MPSA42"/>
<technology name="MPSA44"/>
</technologies>
</device>
<device name="TO92-E1" package="TO92-E1">
<connects>
<connect gate="G$1" pin="B" pad="2"/>
<connect gate="G$1" pin="C" pad="1"/>
<connect gate="G$1" pin="E" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29916/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TO92L" package="TO92L">
<connects>
<connect gate="G$1" pin="B" pad="2"/>
<connect gate="G$1" pin="C" pad="3"/>
<connect gate="G$1" pin="E" pad="1"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29918/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TOP3" package="TOP3">
<connects>
<connect gate="G$1" pin="B" pad="1"/>
<connect gate="G$1" pin="C" pad="2"/>
<connect gate="G$1" pin="E" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28521/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TOP3V" package="TOP3V">
<connects>
<connect gate="G$1" pin="B" pad="B"/>
<connect gate="G$1" pin="C" pad="C"/>
<connect gate="G$1" pin="E" pad="E"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28768/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SOT23-BEC" package="SOT23-BEC">
<connects>
<connect gate="G$1" pin="B" pad="B"/>
<connect gate="G$1" pin="C" pad="C"/>
<connect gate="G$1" pin="E" pad="E"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28748/2"/>
</package3dinstances>
<technologies>
<technology name="BC808"/>
<technology name="BC817"/>
<technology name="BC817-16"/>
<technology name="BC817-16LT1"/>
<technology name="BC817-25"/>
<technology name="BC817-25LT1"/>
<technology name="BC817-40"/>
<technology name="BC817-40LT1"/>
<technology name="BC818"/>
<technology name="BC818-16"/>
<technology name="BC818-25"/>
<technology name="BC818-40"/>
<technology name="BC846"/>
<technology name="BC846A"/>
<technology name="BC846ALT1"/>
<technology name="BC846B"/>
<technology name="BC846BLT1"/>
<technology name="BC847"/>
<technology name="BC847A"/>
<technology name="BC847ALT1"/>
<technology name="BC847B"/>
<technology name="BC847BLT1"/>
<technology name="BC847C"/>
<technology name="BC847CLT1"/>
<technology name="BC848"/>
<technology name="BC848A"/>
<technology name="BC848ALT1"/>
<technology name="BC848B"/>
<technology name="BC848BLT1"/>
<technology name="BC848C"/>
<technology name="BC848CLT1"/>
<technology name="BC849"/>
<technology name="BC850"/>
<technology name="BCF29"/>
<technology name="BCF30"/>
<technology name="BCF32"/>
<technology name="BCF33"/>
<technology name="BCF81"/>
<technology name="BCV71"/>
<technology name="BCV72"/>
<technology name="BCW31"/>
<technology name="BCW32"/>
<technology name="BCW33"/>
<technology name="BCW60"/>
<technology name="BCW71"/>
<technology name="BCW72"/>
<technology name="BCW81"/>
<technology name="BCX19"/>
<technology name="BCX20"/>
<technology name="BCX70"/>
<technology name="BF820"/>
<technology name="BF822"/>
<technology name="BFR53"/>
<technology name="BFR92"/>
<technology name="BFR92A"/>
<technology name="BFR93"/>
<technology name="BFR93A"/>
<technology name="BFS17"/>
<technology name="BFS19"/>
<technology name="BFS20"/>
<technology name="BFT25"/>
<technology name="BSR12"/>
<technology name="BSR13"/>
<technology name="BSR14"/>
<technology name="BSV52LT1"/>
<technology name="MMBT2222ALT1"/>
<technology name="MMBT2369LT1"/>
<technology name="MMBT2484LT1"/>
<technology name="MMBT3904LT1"/>
<technology name="MMBT4401LT1"/>
<technology name="MMBT5551LT1"/>
<technology name="MMBT6429LT1"/>
<technology name="MMBT6517LT1"/>
<technology name="MMBT918LT1"/>
<technology name="MMBTA42LT1"/>
<technology name="MMBTH10LT1"/>
<technology name="MMBTH24LT1"/>
</technologies>
</device>
<device name="SOT23-EBC" package="SOT23-EBC">
<connects>
<connect gate="G$1" pin="B" pad="B"/>
<connect gate="G$1" pin="C" pad="C"/>
<connect gate="G$1" pin="E" pad="E"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28750/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="TO92-BCE" package="TO92-BCE">
<connects>
<connect gate="G$1" pin="B" pad="B"/>
<connect gate="G$1" pin="C" pad="C"/>
<connect gate="G$1" pin="E" pad="E"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29924/2"/>
</package3dinstances>
<technologies>
<technology name="BF420"/>
<technology name="BF422"/>
</technologies>
</device>
<device name="TO92-CBE" package="TO92-CBE">
<connects>
<connect gate="G$1" pin="B" pad="B"/>
<connect gate="G$1" pin="C" pad="C"/>
<connect gate="G$1" pin="E" pad="E"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29921/2"/>
</package3dinstances>
<technologies>
<technology name="2N4124"/>
<technology name="2N5400"/>
<technology name="2N5401"/>
<technology name="2N5550"/>
<technology name="2N5551"/>
<technology name="2N6427"/>
<technology name="2N6517"/>
<technology name="2N6520"/>
<technology name="BC237"/>
<technology name="BC238"/>
<technology name="BC239"/>
<technology name="BC328"/>
<technology name="BC337"/>
<technology name="BC337-16"/>
<technology name="BC337-25"/>
<technology name="BC337-40"/>
<technology name="BC338-16"/>
<technology name="BC338-25"/>
<technology name="BC338-40"/>
<technology name="BC372"/>
<technology name="BC373"/>
<technology name="BC447"/>
<technology name="BC449"/>
<technology name="BC449A"/>
<technology name="BC485"/>
<technology name="BC487"/>
<technology name="BC489"/>
<technology name="BC517"/>
<technology name="BC546"/>
<technology name="BC547"/>
<technology name="BC547A"/>
<technology name="BC547B"/>
<technology name="BC548"/>
<technology name="BC549"/>
<technology name="BC550"/>
<technology name="BC618"/>
<technology name="BF391"/>
<technology name="BF393"/>
<technology name="BF844"/>
<technology name="MPS2222A"/>
<technology name="MPSL51"/>
<technology name="MPSW42"/>
</technologies>
</device>
<device name="TO92-BEC" package="TO92-BEC">
<connects>
<connect gate="G$1" pin="B" pad="B"/>
<connect gate="G$1" pin="C" pad="C"/>
<connect gate="G$1" pin="E" pad="E"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29922/2"/>
</package3dinstances>
<technologies>
<technology name="BF199"/>
<technology name="BF224"/>
<technology name="BF240"/>
</technologies>
</device>
<device name="TO92-CEB" package="TO92-CEB">
<connects>
<connect gate="G$1" pin="B" pad="B"/>
<connect gate="G$1" pin="C" pad="C"/>
<connect gate="G$1" pin="E" pad="E"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29923/2"/>
</package3dinstances>
<technologies>
<technology name="BF374"/>
<technology name="BF959"/>
<technology name="MPHS10"/>
</technologies>
</device>
<device name="SC59-BEC" package="SC59-BEC">
<connects>
<connect gate="G$1" pin="B" pad="B"/>
<connect gate="G$1" pin="C" pad="C"/>
<connect gate="G$1" pin="E" pad="E"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28754/2"/>
</package3dinstances>
<technologies>
<technology name="MMUN2211LT1"/>
<technology name="MMUN2212LT1"/>
<technology name="MMUN2213LT1"/>
<technology name="MMUN2214LT1"/>
</technologies>
</device>
<device name="SOT89-BCE" package="SOT89-BCE">
<connects>
<connect gate="G$1" pin="B" pad="B"/>
<connect gate="G$1" pin="C" pad="C"/>
<connect gate="G$1" pin="E" pad="E"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29925/3"/>
</package3dinstances>
<technologies>
<technology name=""/>
<technology name="BC868"/>
<technology name="BCX54"/>
<technology name="BCX55"/>
<technology name="BCX56"/>
<technology name="BF622"/>
</technologies>
</device>
<device name="SOT-37" package="SOT37">
<connects>
<connect gate="G$1" pin="B" pad="3"/>
<connect gate="G$1" pin="C" pad="1"/>
<connect gate="G$1" pin="E" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:28760/1"/>
</package3dinstances>
<technologies>
<technology name="BFR96"/>
</technologies>
</device>
<device name="TO92-ECB" package="TO92-ECB">
<connects>
<connect gate="G$1" pin="B" pad="B"/>
<connect gate="G$1" pin="C" pad="C"/>
<connect gate="G$1" pin="E" pad="E"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:29929/2"/>
</package3dinstances>
<technologies>
<technology name=""/>
<technology name="BC167"/>
<technology name="BC168"/>
<technology name="BC169"/>
<technology name="BC368"/>
<technology name="BC635"/>
<technology name="BC637"/>
<technology name="BC639"/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="capacitor-wima_mod">
<description>&lt;b&gt;WIMA Capacitors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="C5B7.2" urn="urn:adsk.eagle:footprint:5385/1" locally_modified="yes">
<description>&lt;B&gt;MKS2&lt;/B&gt;, 7.5 x 7.2 mm, grid 5.08 mm</description>
<wire x1="-1.524" y1="0" x2="-0.4572" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.4572" y1="0" x2="-0.4572" y2="0.762" width="0.4064" layer="21"/>
<wire x1="-0.4572" y1="0" x2="-0.4572" y2="-0.762" width="0.4064" layer="21"/>
<wire x1="0.4318" y1="0.762" x2="0.4318" y2="0" width="0.4064" layer="21"/>
<wire x1="0.4318" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.4318" y1="0" x2="0.4318" y2="-0.762" width="0.4064" layer="21"/>
<wire x1="-3.683" y1="3.429" x2="-3.683" y2="-3.429" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-3.683" x2="3.429" y2="-3.683" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-3.429" x2="3.683" y2="3.429" width="0.1524" layer="21"/>
<wire x1="3.429" y1="3.683" x2="-3.429" y2="3.683" width="0.1524" layer="21"/>
<wire x1="3.429" y1="3.683" x2="3.683" y2="3.429" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-3.683" x2="3.683" y2="-3.429" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-3.429" x2="-3.429" y2="-3.683" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="3.429" x2="-3.429" y2="3.683" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="2.1844"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="2.1844"/>
<text x="-2.54" y="4.064" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.175" y="-2.921" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C10B4" urn="urn:adsk.eagle:footprint:5353/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 13.4 x 4 mm, grid 10.16 mm</description>
<wire x1="-3.175" y1="1.27" x2="-3.175" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="0" width="0.4064" layer="21"/>
<wire x1="3.81" y1="0" x2="-2.286" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="0" x2="-2.286" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.81" y1="0" x2="-3.175" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="0" x2="-3.175" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-6.096" y1="2.032" x2="6.096" y2="2.032" width="0.1524" layer="21"/>
<wire x1="6.604" y1="1.524" x2="6.604" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-2.032" x2="-6.096" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-1.524" x2="-6.604" y2="1.524" width="0.1524" layer="21"/>
<wire x1="6.096" y1="2.032" x2="6.604" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="-2.032" x2="6.604" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="-1.524" x2="-6.096" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="1.524" x2="-6.096" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-5.08" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<text x="-3.429" y="2.413" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-1.651" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C10B5" urn="urn:adsk.eagle:footprint:5354/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 13.4 x 5 mm, grid 10.16 mm</description>
<wire x1="-3.175" y1="1.27" x2="-3.175" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="0" width="0.4064" layer="21"/>
<wire x1="3.81" y1="0" x2="-2.286" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="0" x2="-2.286" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.81" y1="0" x2="-3.175" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="0" x2="-3.175" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-6.096" y1="2.54" x2="6.096" y2="2.54" width="0.1524" layer="21"/>
<wire x1="6.604" y1="2.032" x2="6.604" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-2.54" x2="-6.096" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-2.032" x2="-6.604" y2="2.032" width="0.1524" layer="21"/>
<wire x1="6.096" y1="2.54" x2="6.604" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="-2.54" x2="6.604" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="-2.032" x2="-6.096" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="2.032" x2="-6.096" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-5.08" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<text x="-5.08" y="2.794" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-1.905" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C10B6" urn="urn:adsk.eagle:footprint:5355/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 13.4 x 6 mm, grid 10.16 mm</description>
<wire x1="-3.175" y1="1.27" x2="-3.175" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.286" y1="1.27" x2="-2.286" y2="0" width="0.4064" layer="21"/>
<wire x1="3.81" y1="0" x2="-2.286" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="0" x2="-2.286" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-3.81" y1="0" x2="-3.175" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="0" x2="-3.175" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-6.096" y1="3.048" x2="6.096" y2="3.048" width="0.1524" layer="21"/>
<wire x1="6.604" y1="2.54" x2="6.604" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-3.048" x2="-6.096" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-2.54" x2="-6.604" y2="2.54" width="0.1524" layer="21"/>
<wire x1="6.096" y1="3.048" x2="6.604" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="6.096" y1="-3.048" x2="6.604" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="-2.54" x2="-6.096" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-6.604" y1="2.54" x2="-6.096" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-5.08" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<text x="-5.08" y="3.302" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.524" y="-2.032" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C15B5" urn="urn:adsk.eagle:footprint:5356/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 18 x 5 mm, grid 15 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="2.032" x2="9.017" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-2.54" x2="-8.509" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-2.032" x2="-9.017" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="2.54" x2="8.509" y2="2.54" width="0.1524" layer="21"/>
<wire x1="8.509" y1="2.54" x2="9.017" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-2.54" x2="9.017" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-2.032" x2="-8.509" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="2.032" x2="-8.509" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<text x="-7.493" y="2.794" size="1.397" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.032" size="1.397" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C15B6" urn="urn:adsk.eagle:footprint:5357/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 18 x 6 mm, grid 15 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="2.54" x2="9.017" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-3.048" x2="-8.509" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-2.54" x2="-9.017" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="3.048" x2="8.509" y2="3.048" width="0.1524" layer="21"/>
<wire x1="8.509" y1="3.048" x2="9.017" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-3.048" x2="9.017" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-2.54" x2="-8.509" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="2.54" x2="-8.509" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<text x="-7.493" y="3.302" size="1.397" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.032" size="1.397" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C15B7" urn="urn:adsk.eagle:footprint:5358/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 18 x 7 mm, grid 15 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="3.048" x2="9.017" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-3.556" x2="-8.509" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-3.048" x2="-9.017" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="3.556" x2="8.509" y2="3.556" width="0.1524" layer="21"/>
<wire x1="8.509" y1="3.556" x2="9.017" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-3.556" x2="9.017" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-3.048" x2="-8.509" y2="-3.556" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="3.048" x2="-8.509" y2="3.556" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<text x="-7.493" y="3.81" size="1.397" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.286" size="1.397" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C15B8" urn="urn:adsk.eagle:footprint:5359/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 18 x 8 mm, grid 15 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="3.556" x2="9.017" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-4.064" x2="-8.509" y2="-4.064" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-3.556" x2="-9.017" y2="3.556" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="4.064" x2="8.509" y2="4.064" width="0.1524" layer="21"/>
<wire x1="8.509" y1="4.064" x2="9.017" y2="3.556" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-4.064" x2="9.017" y2="-3.556" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-3.556" x2="-8.509" y2="-4.064" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="3.556" x2="-8.509" y2="4.064" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<text x="-7.493" y="4.318" size="1.397" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.54" size="1.397" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C15B9" urn="urn:adsk.eagle:footprint:5360/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 18 x 9 mm, grid 15 mm</description>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="0" width="0.4064" layer="21"/>
<wire x1="-5.08" y1="0" x2="-5.08" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="1.27" x2="-4.191" y2="0" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="-4.191" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-4.191" y1="0" x2="6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0" x2="-6.096" y2="0" width="0.1524" layer="21"/>
<wire x1="9.017" y1="3.937" x2="9.017" y2="-3.937" width="0.1524" layer="21"/>
<wire x1="8.509" y1="-4.445" x2="-8.509" y2="-4.445" width="0.1524" layer="21"/>
<wire x1="-9.017" y1="-3.937" x2="-9.017" y2="3.937" width="0.1524" layer="21"/>
<wire x1="-8.509" y1="4.445" x2="8.509" y2="4.445" width="0.1524" layer="21"/>
<wire x1="8.509" y1="4.445" x2="9.017" y2="3.937" width="0.1524" layer="21" curve="-90"/>
<wire x1="8.509" y1="-4.445" x2="9.017" y2="-3.937" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="-3.937" x2="-8.509" y2="-4.445" width="0.1524" layer="21" curve="90"/>
<wire x1="-9.017" y1="3.937" x2="-8.509" y2="4.445" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-7.493" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<pad name="2" x="7.493" y="0" drill="1.016" diameter="2.159" shape="octagon"/>
<text x="-7.493" y="4.699" size="1.397" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.429" y="-2.54" size="1.397" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C2.5-2" urn="urn:adsk.eagle:footprint:5361/1">
<description>&lt;B&gt;MKS2&lt;/B&gt;, 5 x 2.5 mm, grid 2.54 mm</description>
<wire x1="-2.159" y1="1.27" x2="2.159" y2="1.27" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.27" x2="-2.159" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.016" x2="2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.016" x2="-2.413" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.27" x2="2.413" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.016" x2="-2.159" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.27" x2="2.413" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.016" x2="-2.159" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="1.27" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-1.27" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-1.651" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.651" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C2.5-4" urn="urn:adsk.eagle:footprint:5362/1">
<description>&lt;B&gt;MKS2&lt;/B&gt;, 5 x 4 mm, grid 2.54 mm</description>
<wire x1="-2.159" y1="1.905" x2="2.159" y2="1.905" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.905" x2="-2.159" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.651" x2="2.413" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.651" x2="-2.413" y2="-1.651" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.905" x2="2.413" y2="1.651" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.651" x2="-2.159" y2="1.905" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.905" x2="2.413" y2="-1.651" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.651" x2="-2.159" y2="-1.905" width="0.1524" layer="21" curve="90"/>
<wire x1="1.27" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-1.27" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-1.651" y="2.159" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.651" y="-3.429" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C2.5-5" urn="urn:adsk.eagle:footprint:5363/1">
<description>&lt;B&gt;MKS2&lt;/B&gt;, 5 x 5 mm, grid 2.54 mm</description>
<wire x1="-2.159" y1="2.286" x2="2.159" y2="2.286" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-2.286" x2="-2.159" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="2.413" y1="2.032" x2="2.413" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="2.032" x2="-2.413" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="2.159" y1="2.286" x2="2.413" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="2.032" x2="-2.159" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-2.286" x2="2.413" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-2.032" x2="-2.159" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="1.27" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-1.27" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-1.778" y="2.54" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.778" y="-3.81" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C2.5-6" urn="urn:adsk.eagle:footprint:5364/1">
<description>&lt;B&gt;MKS2&lt;/B&gt;, 5 x 6 mm, grid 2.54 mm</description>
<wire x1="-2.159" y1="2.794" x2="2.159" y2="2.794" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-2.794" x2="-2.159" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="2.413" y1="2.54" x2="2.413" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="2.54" x2="-2.413" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="2.159" y1="2.794" x2="2.413" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="2.54" x2="-2.159" y2="2.794" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-2.794" x2="2.413" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-2.54" x2="-2.159" y2="-2.794" width="0.1524" layer="21" curve="90"/>
<wire x1="1.27" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-1.27" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="2.667" y="0.762" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.905" y="-2.413" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C22.5B10" urn="urn:adsk.eagle:footprint:5365/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 27 x 10 mm, grid 22.5 mm</description>
<wire x1="-12.827" y1="5.334" x2="12.827" y2="5.334" width="0.1524" layer="21"/>
<wire x1="13.335" y1="4.826" x2="13.335" y2="-4.826" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-5.334" x2="-12.827" y2="-5.334" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-4.826" x2="-13.335" y2="4.826" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="5.334" x2="13.335" y2="4.826" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-5.334" x2="13.335" y2="-4.826" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-4.826" x2="-12.827" y2="-5.334" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="4.826" x2="-12.827" y2="5.334" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<text x="-11.303" y="5.588" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C22.5B11" urn="urn:adsk.eagle:footprint:5366/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 27 x 11 mm, grid 22.5 mm</description>
<wire x1="-12.827" y1="5.588" x2="12.827" y2="5.588" width="0.1524" layer="21"/>
<wire x1="13.335" y1="5.08" x2="13.335" y2="-5.08" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-5.588" x2="-12.827" y2="-5.588" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-5.08" x2="-13.335" y2="5.08" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="5.588" x2="13.335" y2="5.08" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-5.588" x2="13.335" y2="-5.08" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-5.08" x2="-12.827" y2="-5.588" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="5.08" x2="-12.827" y2="5.588" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<text x="-11.303" y="5.842" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C22.5B6" urn="urn:adsk.eagle:footprint:5367/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 27 x 6 mm, grid 22.5 mm</description>
<wire x1="-12.827" y1="3.048" x2="12.827" y2="3.048" width="0.1524" layer="21"/>
<wire x1="13.335" y1="2.54" x2="13.335" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-3.048" x2="-12.827" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-2.54" x2="-13.335" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="3.048" x2="13.335" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-3.048" x2="13.335" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-2.54" x2="-12.827" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="2.54" x2="-12.827" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<text x="-11.303" y="3.302" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C22.5B7" urn="urn:adsk.eagle:footprint:5368/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 27 x 7 mm, grid 22.5 mm</description>
<wire x1="-12.827" y1="3.556" x2="12.827" y2="3.556" width="0.1524" layer="21"/>
<wire x1="13.335" y1="3.048" x2="13.335" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-3.556" x2="-12.827" y2="-3.556" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-3.048" x2="-13.335" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="3.556" x2="13.335" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-3.556" x2="13.335" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-3.048" x2="-12.827" y2="-3.556" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="3.048" x2="-12.827" y2="3.556" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<text x="-11.303" y="3.81" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C22.5B8" urn="urn:adsk.eagle:footprint:5369/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 27 x 8 mm, grid 22.5 mm</description>
<wire x1="-12.827" y1="4.318" x2="12.827" y2="4.318" width="0.1524" layer="21"/>
<wire x1="13.335" y1="3.81" x2="13.335" y2="-3.81" width="0.1524" layer="21"/>
<wire x1="12.827" y1="-4.318" x2="-12.827" y2="-4.318" width="0.1524" layer="21"/>
<wire x1="-13.335" y1="-3.81" x2="-13.335" y2="3.81" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="12.827" y1="4.318" x2="13.335" y2="3.81" width="0.1524" layer="21" curve="-90"/>
<wire x1="12.827" y1="-4.318" x2="13.335" y2="-3.81" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="-3.81" x2="-12.827" y2="-4.318" width="0.1524" layer="21" curve="90"/>
<wire x1="-13.335" y1="3.81" x2="-12.827" y2="4.318" width="0.1524" layer="21" curve="-90"/>
<wire x1="-9.652" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="9.652" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-11.303" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<pad name="2" x="11.303" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<text x="-11.303" y="4.572" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C27.5B11" urn="urn:adsk.eagle:footprint:5370/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 31.6 x 11 mm, grid 27.5 mm</description>
<wire x1="-15.24" y1="5.588" x2="15.24" y2="5.588" width="0.1524" layer="21"/>
<wire x1="15.748" y1="5.08" x2="15.748" y2="-5.08" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-5.588" x2="-15.24" y2="-5.588" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-5.08" x2="-15.748" y2="5.08" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="5.588" x2="15.748" y2="5.08" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-5.588" x2="15.748" y2="-5.08" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-5.08" x2="-15.24" y2="-5.588" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="5.08" x2="-15.24" y2="5.588" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" diameter="3.1496" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" diameter="3.1496" shape="octagon"/>
<text x="-13.716" y="5.842" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C27.5B13" urn="urn:adsk.eagle:footprint:5371/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 31.6 x 13 mm, grid 27.5 mm</description>
<wire x1="-15.24" y1="6.604" x2="15.24" y2="6.604" width="0.1524" layer="21"/>
<wire x1="15.748" y1="6.096" x2="15.748" y2="-6.096" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-6.604" x2="-15.24" y2="-6.604" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-6.096" x2="-15.748" y2="6.096" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="6.604" x2="15.748" y2="6.096" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-6.604" x2="15.748" y2="-6.096" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-6.096" x2="-15.24" y2="-6.604" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="6.096" x2="-15.24" y2="6.604" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" diameter="3.1496" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" diameter="3.1496" shape="octagon"/>
<text x="-13.716" y="6.858" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C27.5B15" urn="urn:adsk.eagle:footprint:5372/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 31.6 x 15 mm, grid 27.5 mm</description>
<wire x1="-15.24" y1="7.62" x2="15.24" y2="7.62" width="0.1524" layer="21"/>
<wire x1="15.748" y1="7.112" x2="15.748" y2="-7.112" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-7.62" x2="-15.24" y2="-7.62" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-7.112" x2="-15.748" y2="7.112" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="7.62" x2="15.748" y2="7.112" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-7.62" x2="15.748" y2="-7.112" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-7.112" x2="-15.24" y2="-7.62" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="7.112" x2="-15.24" y2="7.62" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" diameter="3.1496" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" diameter="3.1496" shape="octagon"/>
<text x="-13.716" y="7.874" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C27.5B17" urn="urn:adsk.eagle:footprint:5373/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 31.6 x 17 mm, grid 27.5 mm</description>
<wire x1="-15.24" y1="8.509" x2="15.24" y2="8.509" width="0.1524" layer="21"/>
<wire x1="15.748" y1="8.001" x2="15.748" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-8.509" x2="-15.24" y2="-8.509" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-8.001" x2="-15.748" y2="8.001" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="8.509" x2="15.748" y2="8.001" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-8.509" x2="15.748" y2="-8.001" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-8.001" x2="-15.24" y2="-8.509" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="8.001" x2="-15.24" y2="8.509" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" diameter="3.1496" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" diameter="3.1496" shape="octagon"/>
<text x="-13.716" y="8.763" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C27.5B20" urn="urn:adsk.eagle:footprint:5374/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 31.6 x 20 mm, grid 27.5 mm</description>
<wire x1="-15.24" y1="10.16" x2="15.24" y2="10.16" width="0.1524" layer="21"/>
<wire x1="15.748" y1="9.652" x2="15.748" y2="-9.652" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-10.16" x2="-15.24" y2="-10.16" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-9.652" x2="-15.748" y2="9.652" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="10.16" x2="15.748" y2="9.652" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-10.16" x2="15.748" y2="-9.652" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-9.652" x2="-15.24" y2="-10.16" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="9.652" x2="-15.24" y2="10.16" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" diameter="3.1496" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" diameter="3.1496" shape="octagon"/>
<text x="-13.589" y="10.414" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-4.318" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C27.5B9" urn="urn:adsk.eagle:footprint:5375/1">
<description>&lt;B&gt;MKS4&lt;/B&gt;, 31.6 x 9 mm, grid 27.5 mm</description>
<wire x1="-15.24" y1="4.572" x2="15.24" y2="4.572" width="0.1524" layer="21"/>
<wire x1="15.748" y1="4.064" x2="15.748" y2="-4.064" width="0.1524" layer="21"/>
<wire x1="15.24" y1="-4.572" x2="-15.24" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="-15.748" y1="-4.064" x2="-15.748" y2="4.064" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="15.24" y1="4.572" x2="15.748" y2="4.064" width="0.1524" layer="21" curve="-90"/>
<wire x1="15.24" y1="-4.572" x2="15.748" y2="-4.064" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="-4.064" x2="-15.24" y2="-4.572" width="0.1524" layer="21" curve="90"/>
<wire x1="-15.748" y1="4.064" x2="-15.24" y2="4.572" width="0.1524" layer="21" curve="-90"/>
<wire x1="-11.557" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="11.557" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-13.716" y="0" drill="1.1938" diameter="3.1496" shape="octagon"/>
<pad name="2" x="13.716" y="0" drill="1.1938" diameter="3.1496" shape="octagon"/>
<text x="-13.589" y="4.826" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C37.5B15" urn="urn:adsk.eagle:footprint:5376/1">
<description>&lt;B&gt;MKP4&lt;/B&gt;, 42 x 15 mm, grid 37.5 mm</description>
<wire x1="-20.32" y1="7.62" x2="20.32" y2="7.62" width="0.1524" layer="21"/>
<wire x1="20.828" y1="7.112" x2="20.828" y2="-7.112" width="0.1524" layer="21"/>
<wire x1="20.32" y1="-7.62" x2="-20.32" y2="-7.62" width="0.1524" layer="21"/>
<wire x1="-20.828" y1="-7.112" x2="-20.828" y2="7.112" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="20.32" y1="7.62" x2="20.828" y2="7.112" width="0.1524" layer="21" curve="-90"/>
<wire x1="20.32" y1="-7.62" x2="20.828" y2="-7.112" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="-7.112" x2="-20.32" y2="-7.62" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="7.112" x2="-20.32" y2="7.62" width="0.1524" layer="21" curve="-90"/>
<wire x1="-16.002" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="16.002" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-18.796" y="0" drill="1.3208" diameter="3.1496" shape="octagon"/>
<pad name="2" x="18.796" y="0" drill="1.3208" diameter="3.1496" shape="octagon"/>
<text x="-18.796" y="7.874" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C37.5B19" urn="urn:adsk.eagle:footprint:5377/1">
<description>&lt;B&gt;MKP4&lt;/B&gt;, 42 x 19 mm, grid 37.5 mm</description>
<wire x1="-20.32" y1="8.509" x2="20.32" y2="8.509" width="0.1524" layer="21"/>
<wire x1="20.828" y1="8.001" x2="20.828" y2="-8.001" width="0.1524" layer="21"/>
<wire x1="20.32" y1="-8.509" x2="-20.32" y2="-8.509" width="0.1524" layer="21"/>
<wire x1="-20.828" y1="-8.001" x2="-20.828" y2="8.001" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="20.32" y1="8.509" x2="20.828" y2="8.001" width="0.1524" layer="21" curve="-90"/>
<wire x1="20.32" y1="-8.509" x2="20.828" y2="-8.001" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="-8.001" x2="-20.32" y2="-8.509" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="8.001" x2="-20.32" y2="8.509" width="0.1524" layer="21" curve="-90"/>
<wire x1="-16.002" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="16.002" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-18.796" y="0" drill="1.3208" diameter="3.1496" shape="octagon"/>
<pad name="2" x="18.796" y="0" drill="1.3208" diameter="3.1496" shape="octagon"/>
<text x="-18.796" y="8.89" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C37.5B20" urn="urn:adsk.eagle:footprint:5378/1">
<description>&lt;B&gt;MKP4&lt;/B&gt;, 42 x 20 mm, grid 37.5 mm</description>
<wire x1="-20.32" y1="10.16" x2="20.32" y2="10.16" width="0.1524" layer="21"/>
<wire x1="20.828" y1="9.652" x2="20.828" y2="-9.652" width="0.1524" layer="21"/>
<wire x1="20.32" y1="-10.16" x2="-20.32" y2="-10.16" width="0.1524" layer="21"/>
<wire x1="-20.828" y1="-9.652" x2="-20.828" y2="9.652" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="1.905" x2="-6.731" y2="0" width="0.4064" layer="21"/>
<wire x1="-6.731" y1="0" x2="-6.731" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="1.905" x2="-7.62" y2="0" width="0.4064" layer="21"/>
<wire x1="-7.62" y1="0" x2="-7.62" y2="-1.905" width="0.4064" layer="21"/>
<wire x1="20.32" y1="10.16" x2="20.828" y2="9.652" width="0.1524" layer="21" curve="-90"/>
<wire x1="20.32" y1="-10.16" x2="20.828" y2="-9.652" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="-9.652" x2="-20.32" y2="-10.16" width="0.1524" layer="21" curve="90"/>
<wire x1="-20.828" y1="9.652" x2="-20.32" y2="10.16" width="0.1524" layer="21" curve="-90"/>
<wire x1="-16.002" y1="0" x2="-7.62" y2="0" width="0.1524" layer="21"/>
<wire x1="-6.731" y1="0" x2="16.002" y2="0" width="0.1524" layer="21"/>
<pad name="1" x="-18.796" y="0" drill="1.3208" diameter="3.1496" shape="octagon"/>
<pad name="2" x="18.796" y="0" drill="1.3208" diameter="3.1496" shape="octagon"/>
<text x="-18.796" y="10.414" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C5B2.5" urn="urn:adsk.eagle:footprint:5379/1">
<description>&lt;B&gt;MKS2&lt;/B&gt;, 7.5 x 2.5 mm, grid 5.08 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="1.016" x2="-3.683" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-1.27" x2="3.429" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-1.016" x2="3.683" y2="1.016" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.27" x2="-3.429" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.27" x2="3.683" y2="1.016" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-1.27" x2="3.683" y2="-1.016" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-1.016" x2="-3.429" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="1.016" x2="-3.429" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-2.032" y="1.524" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.032" y="-2.794" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C5B3" urn="urn:adsk.eagle:footprint:5380/1">
<description>&lt;B&gt;MKS2&lt;/B&gt;, 7.5 x 3 mm, grid 5.08 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="1.27" x2="-3.683" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-1.524" x2="3.429" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-1.27" x2="3.683" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.524" x2="-3.429" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.524" x2="3.683" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-1.524" x2="3.683" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-1.27" x2="-3.429" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="1.27" x2="-3.429" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-2.54" y="1.778" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-3.048" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C5B3.5" urn="urn:adsk.eagle:footprint:5381/1">
<description>&lt;B&gt;MKS2&lt;/B&gt;, 7.5 x 4 mm, grid 5.08 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="1.524" x2="-3.683" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-1.778" x2="3.429" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-1.524" x2="3.683" y2="1.524" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.778" x2="-3.429" y2="1.778" width="0.1524" layer="21"/>
<wire x1="3.429" y1="1.778" x2="3.683" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-1.778" x2="3.683" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-1.524" x2="-3.429" y2="-1.778" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="1.524" x2="-3.429" y2="1.778" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-2.54" y="2.032" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-3.302" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C5B4.5" urn="urn:adsk.eagle:footprint:5382/1">
<description>&lt;B&gt;MKS2&lt;/B&gt;, 7.5 x 4.5 mm, grid 5.08 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="2.032" x2="-3.683" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-2.286" x2="3.429" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-2.032" x2="3.683" y2="2.032" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.286" x2="-3.429" y2="2.286" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.286" x2="3.683" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-2.286" x2="3.683" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-2.032" x2="-3.429" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="2.032" x2="-3.429" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-2.54" y="2.54" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-3.81" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C5B5" urn="urn:adsk.eagle:footprint:5383/1">
<description>&lt;B&gt;MKS2&lt;/B&gt;, 7.5 x 5 mm, grid 5.08 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="2.286" x2="-3.683" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-2.54" x2="3.429" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-2.286" x2="3.683" y2="2.286" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.54" x2="-3.429" y2="2.54" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.54" x2="3.683" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-2.54" x2="3.683" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-2.286" x2="-3.429" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="2.286" x2="-3.429" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-2.54" y="2.794" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-2.286" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C5B5.5" urn="urn:adsk.eagle:footprint:5384/1">
<description>&lt;B&gt;MKS2&lt;/B&gt;, 7.5 x 5.5 mm, grid 5.08 mm</description>
<wire x1="-0.3048" y1="0.635" x2="-0.3048" y2="0" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-0.3048" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="-0.3048" y1="0" x2="-1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="0.3302" y1="0.635" x2="0.3302" y2="0" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="0.3302" y2="-0.635" width="0.3048" layer="21"/>
<wire x1="0.3302" y1="0" x2="1.524" y2="0" width="0.1524" layer="21"/>
<wire x1="-3.683" y1="2.54" x2="-3.683" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="-3.429" y1="-2.794" x2="3.429" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="3.683" y1="-2.54" x2="3.683" y2="2.54" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.794" x2="-3.429" y2="2.794" width="0.1524" layer="21"/>
<wire x1="3.429" y1="2.794" x2="3.683" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="3.429" y1="-2.794" x2="3.683" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="-2.54" x2="-3.429" y2="-2.794" width="0.1524" layer="21" curve="90"/>
<wire x1="-3.683" y1="2.54" x2="-3.429" y2="2.794" width="0.1524" layer="21" curve="-90"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-2.54" y="3.048" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-2.286" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C7.5B3" urn="urn:adsk.eagle:footprint:5386/1">
<description>&lt;B&gt;MKS3&lt;/B&gt;, 10 x 3 mm, grid 7.62 mm</description>
<wire x1="4.826" y1="1.524" x2="-4.826" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.27" x2="-5.08" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="-1.524" x2="4.826" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-1.27" x2="5.08" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.826" y1="1.524" x2="5.08" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.826" y1="-1.524" x2="5.08" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="-1.27" x2="-4.826" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.27" x2="-4.826" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="0.508" y1="0" x2="2.54" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="0" x2="-0.508" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.508" y1="0.889" x2="-0.508" y2="0" width="0.4064" layer="21"/>
<wire x1="-0.508" y1="0" x2="-0.508" y2="-0.889" width="0.4064" layer="21"/>
<wire x1="0.508" y1="0.889" x2="0.508" y2="0" width="0.4064" layer="21"/>
<wire x1="0.508" y1="0" x2="0.508" y2="-0.889" width="0.4064" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.9144" diameter="1.905" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.9144" diameter="1.905" shape="octagon"/>
<text x="-3.81" y="1.778" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.81" y="-3.048" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C7.5B4" urn="urn:adsk.eagle:footprint:5387/1">
<description>&lt;B&gt;MKS3&lt;/B&gt;, 10 x 4 mm, grid 7.62 mm</description>
<wire x1="4.826" y1="2.032" x2="-4.826" y2="2.032" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.778" x2="-5.08" y2="-1.778" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="-2.032" x2="4.826" y2="-2.032" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-1.778" x2="5.08" y2="1.778" width="0.1524" layer="21"/>
<wire x1="4.826" y1="2.032" x2="5.08" y2="1.778" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.826" y1="-2.032" x2="5.08" y2="-1.778" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="-1.778" x2="-4.826" y2="-2.032" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.778" x2="-4.826" y2="2.032" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.27" y1="0" x2="2.667" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.667" y1="0" x2="-2.159" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.159" y1="1.27" x2="-2.159" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.159" y1="0" x2="-2.159" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="-1.27" width="0.4064" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.9144" diameter="1.905" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.9144" diameter="1.905" shape="octagon"/>
<text x="-3.81" y="2.286" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.81" y="-3.556" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C7.5B5" urn="urn:adsk.eagle:footprint:5388/1">
<description>&lt;B&gt;MKS3&lt;/B&gt;, 10 x 5 mm, grid 7.62 mm</description>
<wire x1="4.953" y1="2.54" x2="-4.953" y2="2.54" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="2.286" x2="-5.207" y2="-2.286" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="-2.54" x2="4.953" y2="-2.54" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.286" x2="5.207" y2="2.286" width="0.1524" layer="21"/>
<wire x1="4.953" y1="2.54" x2="5.207" y2="2.286" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.953" y1="-2.54" x2="5.207" y2="-2.286" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.207" y1="-2.286" x2="-4.953" y2="-2.54" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.207" y1="2.286" x2="-4.953" y2="2.54" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.27" y1="0" x2="2.667" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.667" y1="0" x2="-2.159" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.159" y1="1.27" x2="-2.159" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.159" y1="0" x2="-2.159" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="-1.27" width="0.4064" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.9144" diameter="1.905" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.9144" diameter="1.905" shape="octagon"/>
<text x="-3.81" y="2.794" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.81" y="-4.064" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C7.5B6" urn="urn:adsk.eagle:footprint:5389/1">
<description>&lt;B&gt;MKS3&lt;/B&gt;, 10 x 6 mm, grid 7.62 mm</description>
<wire x1="4.953" y1="3.048" x2="-4.953" y2="3.048" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="2.794" x2="-5.207" y2="-2.794" width="0.1524" layer="21"/>
<wire x1="-4.953" y1="-3.048" x2="4.953" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="5.207" y1="-2.794" x2="5.207" y2="2.794" width="0.1524" layer="21"/>
<wire x1="4.953" y1="3.048" x2="5.207" y2="2.794" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.953" y1="-3.048" x2="5.207" y2="-2.794" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.207" y1="-2.794" x2="-4.953" y2="-3.048" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.207" y1="2.794" x2="-4.953" y2="3.048" width="0.1524" layer="21" curve="-90"/>
<wire x1="-1.27" y1="0" x2="2.667" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.667" y1="0" x2="-2.159" y2="0" width="0.1524" layer="21"/>
<wire x1="-2.159" y1="1.27" x2="-2.159" y2="0" width="0.4064" layer="21"/>
<wire x1="-2.159" y1="0" x2="-2.159" y2="-1.27" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="1.27" x2="-1.27" y2="0" width="0.4064" layer="21"/>
<wire x1="-1.27" y1="0" x2="-1.27" y2="-1.27" width="0.4064" layer="21"/>
<pad name="1" x="-3.81" y="0" drill="0.9144" diameter="1.905" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.9144" diameter="1.905" shape="octagon"/>
<text x="-3.683" y="3.302" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-0.889" y="-2.667" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="C2.5-3" urn="urn:adsk.eagle:footprint:5390/1">
<description>&lt;B&gt;MKS2&lt;/B&gt;, 5 x 3 mm, grid 2.54 mm</description>
<wire x1="-2.159" y1="1.524" x2="2.159" y2="1.524" width="0.1524" layer="21"/>
<wire x1="2.159" y1="-1.524" x2="-2.159" y2="-1.524" width="0.1524" layer="21"/>
<wire x1="2.413" y1="1.27" x2="2.413" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.413" y1="1.27" x2="-2.413" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.159" y1="1.524" x2="2.413" y2="1.27" width="0.1524" layer="21" curve="-90"/>
<wire x1="-2.413" y1="1.27" x2="-2.159" y2="1.524" width="0.1524" layer="21" curve="-90"/>
<wire x1="2.159" y1="-1.524" x2="2.413" y2="-1.27" width="0.1524" layer="21" curve="90"/>
<wire x1="-2.413" y1="-1.27" x2="-2.159" y2="-1.524" width="0.1524" layer="21" curve="90"/>
<wire x1="1.27" y1="0" x2="0.381" y2="0" width="0.1524" layer="51"/>
<wire x1="0.381" y1="0" x2="0.254" y2="0" width="0.1524" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="0.762" width="0.254" layer="21"/>
<wire x1="0.254" y1="0" x2="0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.254" y2="-0.762" width="0.254" layer="21"/>
<wire x1="-0.254" y1="0" x2="-0.381" y2="0" width="0.1524" layer="21"/>
<wire x1="-0.381" y1="0" x2="-1.27" y2="0" width="0.1524" layer="51"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<pad name="2" x="1.27" y="0" drill="0.8128" diameter="1.6002" shape="octagon"/>
<text x="-1.651" y="1.778" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.651" y="-3.048" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="C5B7.2" urn="urn:adsk.eagle:package:5430/1" locally_modified="yes" type="box">
<description>MKS2, 7.5 x 7.2 mm, grid 5.08 mm</description>
<packageinstances>
<packageinstance name="C5B7.2"/>
</packageinstances>
</package3d>
<package3d name="C10B4" urn="urn:adsk.eagle:package:5400/1" type="box">
<description>MKS4, 13.4 x 4 mm, grid 10.16 mm</description>
<packageinstances>
<packageinstance name="C10B4"/>
</packageinstances>
</package3d>
<package3d name="C10B5" urn="urn:adsk.eagle:package:5399/1" type="box">
<description>MKS4, 13.4 x 5 mm, grid 10.16 mm</description>
<packageinstances>
<packageinstance name="C10B5"/>
</packageinstances>
</package3d>
<package3d name="C10B6" urn="urn:adsk.eagle:package:5401/1" type="box">
<description>MKS4, 13.4 x 6 mm, grid 10.16 mm</description>
<packageinstances>
<packageinstance name="C10B6"/>
</packageinstances>
</package3d>
<package3d name="C15B5" urn="urn:adsk.eagle:package:5402/1" type="box">
<description>MKS4, 18 x 5 mm, grid 15 mm</description>
<packageinstances>
<packageinstance name="C15B5"/>
</packageinstances>
</package3d>
<package3d name="C15B6" urn="urn:adsk.eagle:package:5403/1" type="box">
<description>MKS4, 18 x 6 mm, grid 15 mm</description>
<packageinstances>
<packageinstance name="C15B6"/>
</packageinstances>
</package3d>
<package3d name="C15B7" urn="urn:adsk.eagle:package:5404/1" type="box">
<description>MKS4, 18 x 7 mm, grid 15 mm</description>
<packageinstances>
<packageinstance name="C15B7"/>
</packageinstances>
</package3d>
<package3d name="C15B8" urn="urn:adsk.eagle:package:5409/1" type="box">
<description>MKS4, 18 x 8 mm, grid 15 mm</description>
<packageinstances>
<packageinstance name="C15B8"/>
</packageinstances>
</package3d>
<package3d name="C15B9" urn="urn:adsk.eagle:package:5405/1" type="box">
<description>MKS4, 18 x 9 mm, grid 15 mm</description>
<packageinstances>
<packageinstance name="C15B9"/>
</packageinstances>
</package3d>
<package3d name="C2.5-2" urn="urn:adsk.eagle:package:5415/1" type="box">
<description>MKS2, 5 x 2.5 mm, grid 2.54 mm</description>
<packageinstances>
<packageinstance name="C2.5-2"/>
</packageinstances>
</package3d>
<package3d name="C2.5-4" urn="urn:adsk.eagle:package:5408/1" type="box">
<description>MKS2, 5 x 4 mm, grid 2.54 mm</description>
<packageinstances>
<packageinstance name="C2.5-4"/>
</packageinstances>
</package3d>
<package3d name="C2.5-5" urn="urn:adsk.eagle:package:5407/1" type="box">
<description>MKS2, 5 x 5 mm, grid 2.54 mm</description>
<packageinstances>
<packageinstance name="C2.5-5"/>
</packageinstances>
</package3d>
<package3d name="C2.5-6" urn="urn:adsk.eagle:package:5406/1" type="box">
<description>MKS2, 5 x 6 mm, grid 2.54 mm</description>
<packageinstances>
<packageinstance name="C2.5-6"/>
</packageinstances>
</package3d>
<package3d name="C22.5B10" urn="urn:adsk.eagle:package:5411/1" type="box">
<description>MKS4, 27 x 10 mm, grid 22.5 mm</description>
<packageinstances>
<packageinstance name="C22.5B10"/>
</packageinstances>
</package3d>
<package3d name="C22.5B11" urn="urn:adsk.eagle:package:5412/1" type="box">
<description>MKS4, 27 x 11 mm, grid 22.5 mm</description>
<packageinstances>
<packageinstance name="C22.5B11"/>
</packageinstances>
</package3d>
<package3d name="C22.5B6" urn="urn:adsk.eagle:package:5410/1" type="box">
<description>MKS4, 27 x 6 mm, grid 22.5 mm</description>
<packageinstances>
<packageinstance name="C22.5B6"/>
</packageinstances>
</package3d>
<package3d name="C22.5B7" urn="urn:adsk.eagle:package:5414/1" type="box">
<description>MKS4, 27 x 7 mm, grid 22.5 mm</description>
<packageinstances>
<packageinstance name="C22.5B7"/>
</packageinstances>
</package3d>
<package3d name="C22.5B8" urn="urn:adsk.eagle:package:5413/1" type="box">
<description>MKS4, 27 x 8 mm, grid 22.5 mm</description>
<packageinstances>
<packageinstance name="C22.5B8"/>
</packageinstances>
</package3d>
<package3d name="C27.5B11" urn="urn:adsk.eagle:package:5416/1" type="box">
<description>MKS4, 31.6 x 11 mm, grid 27.5 mm</description>
<packageinstances>
<packageinstance name="C27.5B11"/>
</packageinstances>
</package3d>
<package3d name="C27.5B13" urn="urn:adsk.eagle:package:5420/1" type="box">
<description>MKS4, 31.6 x 13 mm, grid 27.5 mm</description>
<packageinstances>
<packageinstance name="C27.5B13"/>
</packageinstances>
</package3d>
<package3d name="C27.5B15" urn="urn:adsk.eagle:package:5417/1" type="box">
<description>MKS4, 31.6 x 15 mm, grid 27.5 mm</description>
<packageinstances>
<packageinstance name="C27.5B15"/>
</packageinstances>
</package3d>
<package3d name="C27.5B17" urn="urn:adsk.eagle:package:5418/1" type="box">
<description>MKS4, 31.6 x 17 mm, grid 27.5 mm</description>
<packageinstances>
<packageinstance name="C27.5B17"/>
</packageinstances>
</package3d>
<package3d name="C27.5B20" urn="urn:adsk.eagle:package:5421/1" type="box">
<description>MKS4, 31.6 x 20 mm, grid 27.5 mm</description>
<packageinstances>
<packageinstance name="C27.5B20"/>
</packageinstances>
</package3d>
<package3d name="C27.5B9" urn="urn:adsk.eagle:package:5419/1" type="box">
<description>MKS4, 31.6 x 9 mm, grid 27.5 mm</description>
<packageinstances>
<packageinstance name="C27.5B9"/>
</packageinstances>
</package3d>
<package3d name="C37.5B15" urn="urn:adsk.eagle:package:5425/1" type="box">
<description>MKP4, 42 x 15 mm, grid 37.5 mm</description>
<packageinstances>
<packageinstance name="C37.5B15"/>
</packageinstances>
</package3d>
<package3d name="C37.5B19" urn="urn:adsk.eagle:package:5422/1" type="box">
<description>MKP4, 42 x 19 mm, grid 37.5 mm</description>
<packageinstances>
<packageinstance name="C37.5B19"/>
</packageinstances>
</package3d>
<package3d name="C37.5B20" urn="urn:adsk.eagle:package:5423/1" type="box">
<description>MKP4, 42 x 20 mm, grid 37.5 mm</description>
<packageinstances>
<packageinstance name="C37.5B20"/>
</packageinstances>
</package3d>
<package3d name="C5B2.5" urn="urn:adsk.eagle:package:5426/1" type="box">
<description>MKS2, 7.5 x 2.5 mm, grid 5.08 mm</description>
<packageinstances>
<packageinstance name="C5B2.5"/>
</packageinstances>
</package3d>
<package3d name="C5B3" urn="urn:adsk.eagle:package:5433/1" type="box">
<description>MKS2, 7.5 x 3 mm, grid 5.08 mm</description>
<packageinstances>
<packageinstance name="C5B3"/>
</packageinstances>
</package3d>
<package3d name="C5B3.5" urn="urn:adsk.eagle:package:5427/1" type="box">
<description>MKS2, 7.5 x 4 mm, grid 5.08 mm</description>
<packageinstances>
<packageinstance name="C5B3.5"/>
</packageinstances>
</package3d>
<package3d name="C5B4.5" urn="urn:adsk.eagle:package:5424/1" type="box">
<description>MKS2, 7.5 x 4.5 mm, grid 5.08 mm</description>
<packageinstances>
<packageinstance name="C5B4.5"/>
</packageinstances>
</package3d>
<package3d name="C5B5" urn="urn:adsk.eagle:package:5428/1" type="box">
<description>MKS2, 7.5 x 5 mm, grid 5.08 mm</description>
<packageinstances>
<packageinstance name="C5B5"/>
</packageinstances>
</package3d>
<package3d name="C5B5.5" urn="urn:adsk.eagle:package:5429/1" type="box">
<description>MKS2, 7.5 x 5.5 mm, grid 5.08 mm</description>
<packageinstances>
<packageinstance name="C5B5.5"/>
</packageinstances>
</package3d>
<package3d name="C7.5B3" urn="urn:adsk.eagle:package:5434/1" type="box">
<description>MKS3, 10 x 3 mm, grid 7.62 mm</description>
<packageinstances>
<packageinstance name="C7.5B3"/>
</packageinstances>
</package3d>
<package3d name="C7.5B4" urn="urn:adsk.eagle:package:5431/1" type="box">
<description>MKS3, 10 x 4 mm, grid 7.62 mm</description>
<packageinstances>
<packageinstance name="C7.5B4"/>
</packageinstances>
</package3d>
<package3d name="C7.5B5" urn="urn:adsk.eagle:package:5432/1" type="box">
<description>MKS3, 10 x 5 mm, grid 7.62 mm</description>
<packageinstances>
<packageinstance name="C7.5B5"/>
</packageinstances>
</package3d>
<package3d name="C7.5B6" urn="urn:adsk.eagle:package:5435/1" type="box">
<description>MKS3, 10 x 6 mm, grid 7.62 mm</description>
<packageinstances>
<packageinstance name="C7.5B6"/>
</packageinstances>
</package3d>
<package3d name="C2.5-3" urn="urn:adsk.eagle:package:5436/1" type="box">
<description>MKS2, 5 x 3 mm, grid 2.54 mm</description>
<packageinstances>
<packageinstance name="C2.5-3"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="C" urn="urn:adsk.eagle:symbol:5352/1">
<wire x1="0" y1="-2.54" x2="0" y2="-2.032" width="0.1524" layer="94"/>
<wire x1="0" y1="0" x2="0" y2="-0.508" width="0.1524" layer="94"/>
<text x="1.524" y="0.381" size="1.778" layer="95">&gt;NAME</text>
<text x="1.524" y="-4.699" size="1.778" layer="96">&gt;VALUE</text>
<rectangle x1="-2.032" y1="-1.016" x2="2.032" y2="-0.508" layer="94"/>
<rectangle x1="-2.032" y1="-2.032" x2="2.032" y2="-1.524" layer="94"/>
<pin name="1" x="0" y="2.54" visible="off" length="short" direction="pas" swaplevel="1" rot="R270"/>
<pin name="2" x="0" y="-5.08" visible="off" length="short" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="C" urn="urn:adsk.eagle:component:5444/1" locally_modified="yes" prefix="C" uservalue="yes">
<description>&lt;B&gt;CAPACITOR&lt;/B&gt;&lt;p&gt;
naming: grid - package width</description>
<gates>
<gate name="G$1" symbol="C" x="0" y="0"/>
</gates>
<devices>
<device name="10/4" package="C10B4">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5400/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="10/5" package="C10B5">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5399/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="10/6" package="C10B6">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5401/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="15/5" package="C15B5">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5402/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="15/6" package="C15B6">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5403/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="15/7" package="C15B7">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5404/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="15/8" package="C15B8">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5409/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="15/9" package="C15B9">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5405/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="2.5/2" package="C2.5-2">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5415/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="2.5/4" package="C2.5-4">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5408/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="2.5/5" package="C2.5-5">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5407/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="2.5/6" package="C2.5-6">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5406/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="22/10" package="C22.5B10">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5411/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="22/11" package="C22.5B11">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5412/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="22/6" package="C22.5B6">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5410/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="22/7" package="C22.5B7">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5414/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="22/8" package="C22.5B8">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5413/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="27/11" package="C27.5B11">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5416/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="27/13" package="C27.5B13">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5420/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="27/15" package="C27.5B15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5417/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="27/17" package="C27.5B17">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5418/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="27/20" package="C27.5B20">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5421/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="27/9" package="C27.5B9">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5419/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="37/15" package="C37.5B15">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5425/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="37/19" package="C37.5B19">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5422/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="37/20" package="C37.5B20">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5423/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="5/2.5" package="C5B2.5">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5426/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="5/3" package="C5B3">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5433/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="5/3.5" package="C5B3.5">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5427/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="5/4.5" package="C5B4.5">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5424/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="5/5" package="C5B5">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5428/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="5/5.5" package="C5B5.5">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5429/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="5/7.2" package="C5B7.2">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5430/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="7.5/3" package="C7.5B3">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5434/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="7.5/4" package="C7.5B4">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5431/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="7.5/5" package="C7.5B5">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5432/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="7.5/6" package="C7.5B6">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5435/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="2,5-3" package="C2.5-3">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:5436/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="X2" library="jstmod" deviceset="B4B-ZR" device=""/>
<part name="X4" library="con-stocko_mod" deviceset="MKS1852-6-0-202" device="" package3d_urn="urn:adsk.eagle:package:10067/1"/>
<part name="X5" library="con-stocko_mod" deviceset="MKS1852-6-0-202" device="" package3d_urn="urn:adsk.eagle:package:10067/1"/>
<part name="X6" library="con-stocko_mod" deviceset="MKS1852-6-0-202" device="" package3d_urn="urn:adsk.eagle:package:10067/1"/>
<part name="LED1" library="ledmod" deviceset="LED" device="5MM" package3d_urn="urn:adsk.eagle:package:15799/2"/>
<part name="SUPPLY1" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY2" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY3" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY4" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY5" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY6" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="R1" library="eagle-ltspice_mod" deviceset="R" device="0207/10" package3d_urn="urn:adsk.eagle:package:13275/1" value="1K"/>
<part name="R2" library="eagle-ltspice_mod" deviceset="R" device="0207/10" package3d_urn="urn:adsk.eagle:package:13275/1" value="1K"/>
<part name="IC1" library="linear" library_urn="urn:adsk.eagle:library:262" deviceset="78*" device="T" package3d_urn="urn:adsk.eagle:package:16414/1" technology="05"/>
<part name="D1" library="diode_mod" deviceset="1N4004" device="" package3d_urn="urn:adsk.eagle:package:43336/1"/>
<part name="D2" library="diode_mod" deviceset="1N4004" device="" package3d_urn="urn:adsk.eagle:package:43336/1"/>
<part name="C1" library="capacitor-wima_mod" deviceset="C" device="5/7.2" package3d_urn="urn:adsk.eagle:package:5430/1"/>
<part name="C2" library="capacitor-wima_mod" deviceset="C" device="5/7.2" package3d_urn="urn:adsk.eagle:package:5430/1"/>
<part name="X3" library="jstmod" deviceset="B3B-ZR" device=""/>
<part name="SUPPLY9" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY10" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY11" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY12" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY13" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY14" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY15" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="R3" library="eagle-ltspice_mod" deviceset="R" device="0207/10" package3d_urn="urn:adsk.eagle:package:13275/1" value="330"/>
<part name="X1" library="jstmod" deviceset="B3B-ZR" device=""/>
<part name="R4" library="eagle-ltspice_mod" deviceset="R" device="0207/10" package3d_urn="urn:adsk.eagle:package:13275/1" value="10K"/>
<part name="CON2" library="jstmod" deviceset="B5B-ZR" device=""/>
<part name="CON1" library="jst" deviceset="B6B-ZR" device=""/>
<part name="Q1" library="transistor-power" library_urn="urn:adsk.eagle:library:400" deviceset="IRF3704" device="-H" package3d_urn="urn:adsk.eagle:package:29490/1"/>
<part name="Q2" library="transistor-power" library_urn="urn:adsk.eagle:library:400" deviceset="IRF3704" device="-H" package3d_urn="urn:adsk.eagle:package:29490/1"/>
<part name="T3" library="transistor_mod" deviceset="*-NPN-" device="TO92-EBC" package3d_urn="urn:adsk.eagle:package:29915/2"/>
<part name="T4" library="transistor_mod" deviceset="*-NPN-" device="TO92-EBC" package3d_urn="urn:adsk.eagle:package:29915/2"/>
<part name="R5" library="eagle-ltspice_mod" deviceset="R" device="0204/7" package3d_urn="urn:adsk.eagle:package:13274/1" value="10K"/>
<part name="R6" library="eagle-ltspice_mod" deviceset="R" device="0204/7" package3d_urn="urn:adsk.eagle:package:13274/1" value="10K"/>
<part name="SUPPLY7" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="VCC" device=""/>
<part name="SUPPLY8" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="SUPPLY16" library="supply2" library_urn="urn:adsk.eagle:library:372" deviceset="GND" device=""/>
<part name="R7" library="eagle-ltspice_mod" deviceset="R" device="0207/10" package3d_urn="urn:adsk.eagle:package:13275/1" value="10K"/>
<part name="R8" library="eagle-ltspice_mod" deviceset="R" device="0207/10" package3d_urn="urn:adsk.eagle:package:13275/1" value="4.7K"/>
<part name="J1" library="jumper" library_urn="urn:adsk.eagle:library:252" deviceset="J" device="7MM" package3d_urn="urn:adsk.eagle:package:15466/1"/>
</parts>
<sheets>
<sheet>
<plain>
<text x="-7.62" y="-17.78" size="1.778" layer="90">IN +9VDC</text>
<text x="170.18" y="43.18" size="1.778" layer="90">DHT22</text>
<text x="170.18" y="106.68" size="1.778" layer="90">Interruptores llenado &amp; tanque</text>
<text x="170.18" y="76.2" size="1.778" layer="90">Sensor ultrasonico</text>
<text x="160.02" y="12.7" size="1.778" layer="90">Ventilador y resistencia</text>
<text x="160.02" y="-43.18" size="1.778" layer="90">Bomba de agua</text>
<text x="30.48" y="43.18" size="1.778" layer="91">Moteino L2</text>
<text x="27.94" y="106.68" size="1.778" layer="91">Moteino L1</text>
</plain>
<instances>
<instance part="X2" gate="G$1" x="165.1" y="78.74" smashed="yes">
<attribute name="NAME" x="165.1" y="85.09" size="1.778" layer="95"/>
</instance>
<instance part="X4" gate="G$1" x="10.16" y="-17.78" smashed="yes" rot="R180">
<attribute name="NAME" x="12.7" y="-21.59" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="X5" gate="G$1" x="149.86" y="15.24" smashed="yes">
<attribute name="NAME" x="147.32" y="19.05" size="1.778" layer="95"/>
</instance>
<instance part="X6" gate="G$1" x="149.86" y="-40.64" smashed="yes">
<attribute name="NAME" x="147.32" y="-36.83" size="1.778" layer="95"/>
</instance>
<instance part="LED1" gate="G$1" x="78.74" y="38.1" smashed="yes">
<attribute name="NAME" x="82.296" y="33.528" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="84.455" y="33.528" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY1" gate="GND" x="119.38" y="-15.24" smashed="yes">
<attribute name="VALUE" x="117.475" y="-18.415" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY2" gate="GND" x="119.38" y="-66.04" smashed="yes">
<attribute name="VALUE" x="117.475" y="-69.215" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY3" gate="GND" x="58.42" y="-33.02" smashed="yes">
<attribute name="VALUE" x="56.515" y="-36.195" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY4" gate="GND" x="27.94" y="-33.02" smashed="yes">
<attribute name="VALUE" x="26.035" y="-36.195" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY5" gate="G$1" x="73.66" y="-10.16" smashed="yes">
<attribute name="VALUE" x="71.755" y="-6.985" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY6" gate="G$1" x="137.16" y="-30.48" smashed="yes">
<attribute name="VALUE" x="135.255" y="-27.305" size="1.778" layer="96"/>
</instance>
<instance part="R1" gate="G$1" x="101.6" y="-5.08" smashed="yes">
<attribute name="NAME" x="97.79" y="-3.5814" size="1.778" layer="95"/>
<attribute name="VALUE" x="100.33" y="-8.382" size="1.778" layer="96"/>
</instance>
<instance part="R2" gate="G$1" x="104.14" y="-55.88" smashed="yes">
<attribute name="NAME" x="100.33" y="-54.3814" size="1.778" layer="95"/>
<attribute name="VALUE" x="102.87" y="-59.182" size="1.778" layer="96"/>
</instance>
<instance part="IC1" gate="A1" x="58.42" y="-15.24" smashed="yes">
<attribute name="NAME" x="50.8" y="-9.525" size="1.778" layer="95"/>
<attribute name="VALUE" x="50.8" y="-12.065" size="1.778" layer="96"/>
</instance>
<instance part="D1" gate="1" x="137.16" y="15.24" smashed="yes" rot="R90">
<attribute name="NAME" x="136.6774" y="17.78" size="1.778" layer="95" rot="R90"/>
</instance>
<instance part="D2" gate="1" x="137.16" y="-40.64" smashed="yes" rot="R90">
<attribute name="NAME" x="136.6774" y="-38.1" size="1.778" layer="95" rot="R90"/>
</instance>
<instance part="C1" gate="G$1" x="73.66" y="-20.32" smashed="yes">
<attribute name="NAME" x="77.724" y="-19.939" size="1.778" layer="95"/>
<attribute name="VALUE" x="75.184" y="-25.019" size="1.778" layer="96"/>
</instance>
<instance part="C2" gate="G$1" x="43.18" y="-20.32" smashed="yes">
<attribute name="NAME" x="44.704" y="-19.939" size="1.778" layer="95"/>
<attribute name="VALUE" x="44.704" y="-25.019" size="1.778" layer="96"/>
</instance>
<instance part="X3" gate="G$1" x="165.1" y="43.18" smashed="yes">
<attribute name="NAME" x="162.56" y="46.99" size="1.778" layer="95"/>
</instance>
<instance part="SUPPLY9" gate="G$1" x="149.86" y="96.52" smashed="yes">
<attribute name="VALUE" x="147.955" y="99.695" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY10" gate="G$1" x="55.88" y="53.34" smashed="yes">
<attribute name="VALUE" x="53.975" y="56.515" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY11" gate="GND" x="55.88" y="33.02" smashed="yes">
<attribute name="VALUE" x="53.975" y="29.845" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY12" gate="GND" x="157.48" y="35.56" smashed="yes">
<attribute name="VALUE" x="155.575" y="32.385" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY13" gate="GND" x="157.48" y="101.6" smashed="yes">
<attribute name="VALUE" x="155.575" y="98.425" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY14" gate="GND" x="154.94" y="55.88" smashed="yes">
<attribute name="VALUE" x="158.115" y="55.245" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY15" gate="GND" x="78.74" y="27.94" smashed="yes">
<attribute name="VALUE" x="81.915" y="24.765" size="1.778" layer="96"/>
</instance>
<instance part="R3" gate="G$1" x="71.12" y="43.18" smashed="yes">
<attribute name="NAME" x="67.31" y="44.6786" size="1.778" layer="95"/>
<attribute name="VALUE" x="69.85" y="39.878" size="1.778" layer="96"/>
</instance>
<instance part="X1" gate="G$1" x="165.1" y="109.22" smashed="yes">
<attribute name="NAME" x="165.1" y="115.57" size="1.778" layer="95"/>
</instance>
<instance part="R4" gate="G$1" x="121.92" y="55.88" smashed="yes" rot="R180">
<attribute name="NAME" x="120.65" y="59.4614" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="123.19" y="54.102" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="CON2" gate="G$1" x="45.72" y="43.18" smashed="yes" rot="MR0">
<attribute name="NAME" x="48.26" y="51.562" size="1.27" layer="95" rot="MR0"/>
</instance>
<instance part="CON1" gate="G$1" x="48.26" y="111.76" smashed="yes" rot="MR0">
<attribute name="NAME" x="48.26" y="115.062" size="1.27" layer="95" rot="MR0"/>
</instance>
<instance part="Q1" gate="G$1" x="134.62" y="-55.88" smashed="yes">
<attribute name="NAME" x="140.97" y="-53.34" size="1.778" layer="95"/>
</instance>
<instance part="Q2" gate="G$1" x="134.62" y="-2.54" smashed="yes">
<attribute name="NAME" x="140.97" y="0" size="1.778" layer="95"/>
</instance>
<instance part="T3" gate="G$1" x="116.84" y="-5.08" smashed="yes">
<attribute name="NAME" x="106.68" y="2.54" size="1.778" layer="95"/>
<attribute name="VALUE" x="106.68" y="0" size="1.778" layer="96"/>
</instance>
<instance part="T4" gate="G$1" x="116.84" y="-55.88" smashed="yes">
<attribute name="NAME" x="106.68" y="-48.26" size="1.778" layer="95"/>
<attribute name="VALUE" x="106.68" y="-50.8" size="1.778" layer="96"/>
</instance>
<instance part="R5" gate="G$1" x="119.38" y="10.16" smashed="yes" rot="R90">
<attribute name="NAME" x="117.8814" y="13.97" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="122.682" y="6.35" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="R6" gate="G$1" x="119.38" y="-40.64" smashed="yes" rot="R90">
<attribute name="NAME" x="117.8814" y="-36.83" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="122.682" y="-41.91" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUPPLY7" gate="G$1" x="119.38" y="-30.48" smashed="yes">
<attribute name="VALUE" x="117.475" y="-27.305" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY8" gate="GND" x="137.16" y="-15.24" smashed="yes">
<attribute name="VALUE" x="135.255" y="-18.415" size="1.778" layer="96"/>
</instance>
<instance part="SUPPLY16" gate="GND" x="137.16" y="-66.04" smashed="yes">
<attribute name="VALUE" x="135.255" y="-69.215" size="1.778" layer="96"/>
</instance>
<instance part="R7" gate="G$1" x="134.62" y="68.58" smashed="yes" rot="R270">
<attribute name="NAME" x="131.0386" y="67.31" size="1.778" layer="95" rot="R270"/>
<attribute name="VALUE" x="136.398" y="69.85" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="R8" gate="G$1" x="142.24" y="78.74" smashed="yes" rot="R180">
<attribute name="NAME" x="140.97" y="82.3214" size="1.778" layer="95" rot="R180"/>
<attribute name="VALUE" x="143.51" y="76.962" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="J1" gate="1" x="149.86" y="86.36" smashed="yes" rot="R90">
<attribute name="NAME" x="147.955" y="83.82" size="1.778" layer="95" rot="R90"/>
<attribute name="VALUE" x="152.4" y="83.82" size="1.778" layer="96" rot="R90"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="N$1" class="0">
<segment>
<wire x1="109.22" y1="-55.88" x2="114.3" y2="-55.88" width="0.1524" layer="91"/>
<pinref part="R2" gate="G$1" pin="2"/>
<pinref part="T4" gate="G$1" pin="B"/>
</segment>
</net>
<net name="GND" class="0">
<segment>
<pinref part="SUPPLY2" gate="GND" pin="GND"/>
<wire x1="119.38" y1="-60.96" x2="119.38" y2="-63.5" width="0.1524" layer="91"/>
<pinref part="T4" gate="G$1" pin="E"/>
</segment>
<segment>
<pinref part="SUPPLY1" gate="GND" pin="GND"/>
<pinref part="T3" gate="G$1" pin="E"/>
<wire x1="119.38" y1="-12.7" x2="119.38" y2="-10.16" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C1" gate="G$1" pin="2"/>
<wire x1="73.66" y1="-25.4" x2="58.42" y2="-25.4" width="0.1524" layer="91"/>
<pinref part="IC1" gate="A1" pin="GND"/>
<wire x1="58.42" y1="-25.4" x2="58.42" y2="-22.86" width="0.1524" layer="91"/>
<pinref part="SUPPLY3" gate="GND" pin="GND"/>
<wire x1="58.42" y1="-25.4" x2="58.42" y2="-30.48" width="0.1524" layer="91"/>
<junction x="58.42" y="-25.4"/>
<pinref part="C2" gate="G$1" pin="2"/>
<wire x1="58.42" y1="-25.4" x2="43.18" y2="-25.4" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="X4" gate="G$1" pin="1"/>
<wire x1="15.24" y1="-17.78" x2="27.94" y2="-17.78" width="0.1524" layer="91"/>
<wire x1="27.94" y1="-17.78" x2="27.94" y2="-30.48" width="0.1524" layer="91"/>
<pinref part="SUPPLY4" gate="GND" pin="GND"/>
</segment>
<segment>
<pinref part="LED1" gate="G$1" pin="C"/>
<pinref part="SUPPLY15" gate="GND" pin="GND"/>
<wire x1="78.74" y1="33.02" x2="78.74" y2="30.48" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SUPPLY13" gate="GND" pin="GND"/>
<wire x1="157.48" y1="104.14" x2="157.48" y2="106.68" width="0.1524" layer="91"/>
<pinref part="X1" gate="G$1" pin="3"/>
</segment>
<segment>
<pinref part="X3" gate="G$1" pin="3"/>
<pinref part="SUPPLY12" gate="GND" pin="GND"/>
<wire x1="157.48" y1="40.64" x2="157.48" y2="38.1" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SUPPLY11" gate="GND" pin="GND"/>
<wire x1="53.34" y1="38.1" x2="55.88" y2="38.1" width="0.1524" layer="91"/>
<wire x1="55.88" y1="38.1" x2="55.88" y2="35.56" width="0.1524" layer="91"/>
<pinref part="CON2" gate="G$1" pin="5"/>
</segment>
<segment>
<pinref part="Q1" gate="G$1" pin="S"/>
<pinref part="SUPPLY16" gate="GND" pin="GND"/>
<wire x1="137.16" y1="-60.96" x2="137.16" y2="-63.5" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SUPPLY8" gate="GND" pin="GND"/>
<pinref part="Q2" gate="G$1" pin="S"/>
<wire x1="137.16" y1="-12.7" x2="137.16" y2="-7.62" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SUPPLY14" gate="GND" pin="GND"/>
<pinref part="X2" gate="G$1" pin="1"/>
<wire x1="154.94" y1="58.42" x2="154.94" y2="60.96" width="0.1524" layer="91"/>
<wire x1="154.94" y1="60.96" x2="154.94" y2="81.28" width="0.1524" layer="91"/>
<wire x1="154.94" y1="81.28" x2="157.48" y2="81.28" width="0.1524" layer="91"/>
<pinref part="R7" gate="G$1" pin="2"/>
<wire x1="134.62" y1="63.5" x2="134.62" y2="60.96" width="0.1524" layer="91"/>
<wire x1="134.62" y1="60.96" x2="154.94" y2="60.96" width="0.1524" layer="91"/>
<junction x="154.94" y="60.96"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="R1" gate="G$1" pin="2"/>
<wire x1="106.68" y1="-5.08" x2="114.3" y2="-5.08" width="0.1524" layer="91"/>
<pinref part="T3" gate="G$1" pin="B"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="D1" gate="1" pin="A"/>
<wire x1="137.16" y1="12.7" x2="137.16" y2="10.16" width="0.1524" layer="91"/>
<wire x1="137.16" y1="10.16" x2="137.16" y2="2.54" width="0.1524" layer="91"/>
<wire x1="137.16" y1="10.16" x2="144.78" y2="10.16" width="0.1524" layer="91"/>
<junction x="137.16" y="10.16"/>
<wire x1="144.78" y1="10.16" x2="144.78" y2="12.7" width="0.1524" layer="91"/>
<pinref part="X5" gate="G$1" pin="2"/>
<pinref part="Q2" gate="G$1" pin="D"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="D2" gate="1" pin="A"/>
<wire x1="137.16" y1="-43.18" x2="137.16" y2="-45.72" width="0.1524" layer="91"/>
<wire x1="137.16" y1="-45.72" x2="144.78" y2="-45.72" width="0.1524" layer="91"/>
<wire x1="144.78" y1="-45.72" x2="144.78" y2="-43.18" width="0.1524" layer="91"/>
<junction x="137.16" y="-45.72"/>
<pinref part="X6" gate="G$1" pin="2"/>
<pinref part="Q1" gate="G$1" pin="D"/>
<wire x1="137.16" y1="-50.8" x2="137.16" y2="-45.72" width="0.1524" layer="91"/>
</segment>
</net>
<net name="VCC" class="0">
<segment>
<pinref part="IC1" gate="A1" pin="VO"/>
<wire x1="68.58" y1="-15.24" x2="73.66" y2="-15.24" width="0.1524" layer="91"/>
<pinref part="C1" gate="G$1" pin="1"/>
<wire x1="73.66" y1="-15.24" x2="73.66" y2="-17.78" width="0.1524" layer="91"/>
<pinref part="SUPPLY5" gate="G$1" pin="VCC"/>
<wire x1="73.66" y1="-15.24" x2="73.66" y2="-12.7" width="0.1524" layer="91"/>
<junction x="73.66" y="-15.24"/>
</segment>
<segment>
<pinref part="SUPPLY6" gate="G$1" pin="VCC"/>
<pinref part="D2" gate="1" pin="C"/>
<wire x1="137.16" y1="-33.02" x2="137.16" y2="-35.56" width="0.1524" layer="91"/>
<pinref part="X6" gate="G$1" pin="1"/>
<wire x1="137.16" y1="-35.56" x2="137.16" y2="-38.1" width="0.1524" layer="91"/>
<wire x1="144.78" y1="-40.64" x2="144.78" y2="-35.56" width="0.1524" layer="91"/>
<wire x1="144.78" y1="-35.56" x2="137.16" y2="-35.56" width="0.1524" layer="91"/>
<junction x="137.16" y="-35.56"/>
</segment>
<segment>
<pinref part="SUPPLY10" gate="G$1" pin="VCC"/>
<wire x1="55.88" y1="40.64" x2="55.88" y2="50.8" width="0.1524" layer="91"/>
<wire x1="53.34" y1="40.64" x2="55.88" y2="40.64" width="0.1524" layer="91"/>
<pinref part="CON2" gate="G$1" pin="4"/>
</segment>
<segment>
<pinref part="R6" gate="G$1" pin="2"/>
<pinref part="SUPPLY7" gate="G$1" pin="VCC"/>
<wire x1="119.38" y1="-33.02" x2="119.38" y2="-35.56" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SUPPLY9" gate="G$1" pin="VCC"/>
<pinref part="J1" gate="1" pin="2"/>
<wire x1="149.86" y1="93.98" x2="149.86" y2="91.44" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="R3" gate="G$1" pin="2"/>
<wire x1="76.2" y1="43.18" x2="78.74" y2="43.18" width="0.1524" layer="91"/>
<pinref part="LED1" gate="G$1" pin="A"/>
<wire x1="78.74" y1="43.18" x2="78.74" y2="40.64" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<wire x1="157.48" y1="111.76" x2="53.34" y2="111.76" width="0.1524" layer="91"/>
<pinref part="X1" gate="G$1" pin="1"/>
<pinref part="CON1" gate="G$1" pin="1"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<wire x1="157.48" y1="109.22" x2="53.34" y2="109.22" width="0.1524" layer="91"/>
<pinref part="X1" gate="G$1" pin="2"/>
<pinref part="CON1" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$14" class="0">
<segment>
<pinref part="R1" gate="G$1" pin="1"/>
<wire x1="93.98" y1="48.26" x2="93.98" y2="-5.08" width="0.1524" layer="91"/>
<wire x1="93.98" y1="-5.08" x2="96.52" y2="-5.08" width="0.1524" layer="91"/>
<wire x1="53.34" y1="48.26" x2="93.98" y2="48.26" width="0.1524" layer="91"/>
<pinref part="CON2" gate="G$1" pin="1"/>
</segment>
</net>
<net name="N$15" class="0">
<segment>
<pinref part="R2" gate="G$1" pin="1"/>
<wire x1="88.9" y1="45.72" x2="88.9" y2="-55.88" width="0.1524" layer="91"/>
<wire x1="88.9" y1="-55.88" x2="99.06" y2="-55.88" width="0.1524" layer="91"/>
<wire x1="53.34" y1="45.72" x2="88.9" y2="45.72" width="0.1524" layer="91"/>
<pinref part="CON2" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$16" class="0">
<segment>
<pinref part="D1" gate="1" pin="C"/>
<pinref part="X5" gate="G$1" pin="1"/>
<wire x1="137.16" y1="20.32" x2="137.16" y2="17.78" width="0.1524" layer="91"/>
<wire x1="144.78" y1="15.24" x2="144.78" y2="20.32" width="0.1524" layer="91"/>
<wire x1="144.78" y1="20.32" x2="137.16" y2="20.32" width="0.1524" layer="91"/>
<wire x1="137.16" y1="20.32" x2="119.38" y2="20.32" width="0.1524" layer="91"/>
<junction x="137.16" y="20.32"/>
<pinref part="C2" gate="G$1" pin="1"/>
<wire x1="119.38" y1="20.32" x2="27.94" y2="20.32" width="0.1524" layer="91"/>
<wire x1="43.18" y1="-17.78" x2="43.18" y2="-15.24" width="0.1524" layer="91"/>
<pinref part="IC1" gate="A1" pin="VI"/>
<wire x1="43.18" y1="-15.24" x2="48.26" y2="-15.24" width="0.1524" layer="91"/>
<pinref part="X4" gate="G$1" pin="2"/>
<wire x1="15.24" y1="-15.24" x2="27.94" y2="-15.24" width="0.1524" layer="91"/>
<junction x="43.18" y="-15.24"/>
<wire x1="27.94" y1="-15.24" x2="43.18" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="27.94" y1="20.32" x2="27.94" y2="-15.24" width="0.1524" layer="91"/>
<junction x="27.94" y="-15.24"/>
<pinref part="R5" gate="G$1" pin="2"/>
<wire x1="119.38" y1="20.32" x2="119.38" y2="15.24" width="0.1524" layer="91"/>
<junction x="119.38" y="20.32"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<pinref part="X2" gate="G$1" pin="3"/>
<wire x1="157.48" y1="76.2" x2="71.12" y2="76.2" width="0.1524" layer="91"/>
<wire x1="71.12" y1="76.2" x2="71.12" y2="104.14" width="0.1524" layer="91"/>
<wire x1="71.12" y1="104.14" x2="53.34" y2="104.14" width="0.1524" layer="91"/>
<pinref part="CON1" gate="G$1" pin="4"/>
</segment>
</net>
<net name="N$12" class="0">
<segment>
<wire x1="73.66" y1="106.68" x2="73.66" y2="78.74" width="0.1524" layer="91"/>
<wire x1="53.34" y1="106.68" x2="73.66" y2="106.68" width="0.1524" layer="91"/>
<pinref part="CON1" gate="G$1" pin="3"/>
<pinref part="R8" gate="G$1" pin="2"/>
<wire x1="73.66" y1="78.74" x2="134.62" y2="78.74" width="0.1524" layer="91"/>
<pinref part="R7" gate="G$1" pin="1"/>
<wire x1="134.62" y1="78.74" x2="137.16" y2="78.74" width="0.1524" layer="91"/>
<wire x1="134.62" y1="73.66" x2="134.62" y2="78.74" width="0.1524" layer="91"/>
<junction x="134.62" y="78.74"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<pinref part="R3" gate="G$1" pin="1"/>
<wire x1="53.34" y1="43.18" x2="66.04" y2="43.18" width="0.1524" layer="91"/>
<pinref part="CON2" gate="G$1" pin="3"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<wire x1="68.58" y1="101.6" x2="68.58" y2="73.66" width="0.1524" layer="91"/>
<wire x1="68.58" y1="73.66" x2="127" y2="73.66" width="0.1524" layer="91"/>
<wire x1="53.34" y1="101.6" x2="68.58" y2="101.6" width="0.1524" layer="91"/>
<wire x1="127" y1="73.66" x2="127" y2="55.88" width="0.1524" layer="91"/>
<pinref part="R4" gate="G$1" pin="1"/>
<junction x="127" y="55.88"/>
<pinref part="CON1" gate="G$1" pin="5"/>
<pinref part="X3" gate="G$1" pin="1"/>
<wire x1="157.48" y1="45.72" x2="127" y2="45.72" width="0.1524" layer="91"/>
<wire x1="127" y1="45.72" x2="127" y2="55.88" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<wire x1="116.84" y1="43.18" x2="116.84" y2="55.88" width="0.1524" layer="91"/>
<wire x1="116.84" y1="55.88" x2="116.84" y2="71.12" width="0.1524" layer="91"/>
<wire x1="116.84" y1="71.12" x2="66.04" y2="71.12" width="0.1524" layer="91"/>
<wire x1="66.04" y1="71.12" x2="66.04" y2="99.06" width="0.1524" layer="91"/>
<wire x1="66.04" y1="99.06" x2="53.34" y2="99.06" width="0.1524" layer="91"/>
<pinref part="R4" gate="G$1" pin="2"/>
<junction x="116.84" y="55.88"/>
<pinref part="CON1" gate="G$1" pin="6"/>
<pinref part="X3" gate="G$1" pin="2"/>
<wire x1="157.48" y1="43.18" x2="116.84" y2="43.18" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<wire x1="127" y1="2.54" x2="127" y2="-5.08" width="0.1524" layer="91"/>
<pinref part="Q2" gate="G$1" pin="G"/>
<wire x1="127" y1="-5.08" x2="132.08" y2="-5.08" width="0.1524" layer="91"/>
<pinref part="T3" gate="G$1" pin="C"/>
<pinref part="R5" gate="G$1" pin="1"/>
<wire x1="119.38" y1="5.08" x2="119.38" y2="2.54" width="0.1524" layer="91"/>
<wire x1="119.38" y1="2.54" x2="119.38" y2="0" width="0.1524" layer="91"/>
<wire x1="127" y1="2.54" x2="119.38" y2="2.54" width="0.1524" layer="91"/>
<junction x="119.38" y="2.54"/>
</segment>
</net>
<net name="N$17" class="0">
<segment>
<wire x1="127" y1="-48.26" x2="127" y2="-58.42" width="0.1524" layer="91"/>
<pinref part="Q1" gate="G$1" pin="G"/>
<wire x1="127" y1="-58.42" x2="132.08" y2="-58.42" width="0.1524" layer="91"/>
<pinref part="T4" gate="G$1" pin="C"/>
<pinref part="R6" gate="G$1" pin="1"/>
<wire x1="119.38" y1="-45.72" x2="119.38" y2="-48.26" width="0.1524" layer="91"/>
<wire x1="119.38" y1="-48.26" x2="119.38" y2="-50.8" width="0.1524" layer="91"/>
<wire x1="127" y1="-48.26" x2="119.38" y2="-48.26" width="0.1524" layer="91"/>
<junction x="119.38" y="-48.26"/>
</segment>
</net>
<net name="N$18" class="0">
<segment>
<pinref part="R8" gate="G$1" pin="1"/>
<pinref part="X2" gate="G$1" pin="2"/>
<wire x1="147.32" y1="78.74" x2="157.48" y2="78.74" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$19" class="0">
<segment>
<pinref part="X2" gate="G$1" pin="4"/>
<wire x1="149.86" y1="73.66" x2="157.48" y2="73.66" width="0.1524" layer="91"/>
<pinref part="J1" gate="1" pin="1"/>
<wire x1="149.86" y1="81.28" x2="149.86" y2="73.66" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports the association of 3D packages
with devices in libraries, schematics, and board files. Those 3D
packages will not be understood (or retained) with this version.
</note>
</compatibility>
</eagle>
