(kicad_sch
	(version 20231120)
	(generator "eeschema")
	(generator_version "8.0")
	(uuid "84fbcbd8-9b55-4c71-8663-267634a18e15")
	(paper "A4")
	(lib_symbols
		(symbol "Conn_01x02_Socket_1"
			(pin_names
				(offset 1.016) hide)
			(exclude_from_sim no)
			(in_bom yes)
			(on_board yes)
			(property "Reference" "J"
				(at 0 2.54 0)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "Value" "Conn_01x02_Socket"
				(at 0 -5.08 0)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "Footprint" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Datasheet" "~"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Description" "Generic connector, single row, 01x02, script generated"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_locked" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "ki_keywords" "connector"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_fp_filters" "Connector*:*_1x??_*"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(symbol "Conn_01x02_Socket_1_1_1"
				(arc
					(start 0 -2.032)
					(mid -0.5058 -2.54)
					(end 0 -3.048)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy -1.27 -2.54) (xy -0.508 -2.54)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy -1.27 0) (xy -0.508 0)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(arc
					(start 0 0.508)
					(mid -0.5058 0)
					(end 0 -0.508)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(pin passive line
					(at -5.08 0 0)
					(length 3.81)
					(name "Pin_1"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "1"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at -5.08 -2.54 0)
					(length 3.81)
					(name "Pin_2"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "2"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
			)
		)
		(symbol "Conn_01x10_Pin_1"
			(pin_names
				(offset 1.016) hide)
			(exclude_from_sim no)
			(in_bom yes)
			(on_board yes)
			(property "Reference" "Bateria1"
				(at -5.08 8.89 90)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "Value" "Entrada bateria"
				(at -2.54 9.144 90)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "Footprint" "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Datasheet" "~"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Description" "Generic connector, single row, 01x10, script generated"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_locked" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "ki_keywords" "connector"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_fp_filters" "Connector*:*_1x??_*"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(symbol "Conn_01x10_Pin_1_1_1"
				(polyline
					(pts
						(xy 1.27 7.62) (xy 0.8636 7.62)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy 1.27 10.16) (xy 0.8636 10.16)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(rectangle
					(start 0.8636 7.747)
					(end 0 7.493)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type outline)
					)
				)
				(rectangle
					(start 0.8636 10.287)
					(end 0 10.033)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type outline)
					)
				)
				(pin passive line
					(at 5.08 10.16 180)
					(length 3.81)
					(name "+"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "1"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at 5.08 7.62 180)
					(length 3.81)
					(name "-"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "2"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
			)
		)
		(symbol "Connector:Conn_01x01_Socket"
			(pin_names
				(offset 1.016) hide)
			(exclude_from_sim no)
			(in_bom yes)
			(on_board yes)
			(property "Reference" "J"
				(at 0 2.54 0)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "Value" "Conn_01x01_Socket"
				(at 0 -2.54 0)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "Footprint" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Datasheet" "~"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Description" "Generic connector, single row, 01x01, script generated"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_locked" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "ki_keywords" "connector"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_fp_filters" "Connector*:*_1x??_*"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(symbol "Conn_01x01_Socket_1_1"
				(polyline
					(pts
						(xy -1.27 0) (xy -0.508 0)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(arc
					(start 0 0.508)
					(mid -0.5058 0)
					(end 0 -0.508)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(pin passive line
					(at -5.08 0 0)
					(length 3.81)
					(name "Pin_1"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "1"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
			)
		)
		(symbol "Connector:Conn_01x02_Socket"
			(pin_names
				(offset 1.016) hide)
			(exclude_from_sim no)
			(in_bom yes)
			(on_board yes)
			(property "Reference" "J2"
				(at 1.27 0.0001 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify left)
				)
			)
			(property "Value" "Motor 1"
				(at 1.27 -2.5399 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify left)
				)
			)
			(property "Footprint" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Datasheet" "~"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Description" "Generic connector, single row, 01x02, script generated"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_locked" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "ki_keywords" "connector"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_fp_filters" "Connector*:*_1x??_*"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(symbol "Conn_01x02_Socket_1_1"
				(arc
					(start 0 -2.032)
					(mid -0.5058 -2.54)
					(end 0 -3.048)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy -1.27 -2.54) (xy -0.508 -2.54)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy -1.27 0) (xy -0.508 0)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(arc
					(start 0 0.508)
					(mid -0.5058 0)
					(end 0 -0.508)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(pin passive line
					(at -5.08 0 0)
					(length 3.81)
					(name "+"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "1"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at -5.08 -2.54 0)
					(length 3.81)
					(name "-"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "2"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
			)
		)
		(symbol "Connector:Conn_01x04_Socket"
			(pin_names
				(offset 1.016) hide)
			(exclude_from_sim no)
			(in_bom yes)
			(on_board yes)
			(property "Reference" "J5"
				(at 5.08 -1.27 90)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "Value" "Sensor Lateral - Esquerda"
				(at 2.54 -1.27 90)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "Footprint" "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Datasheet" "~"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Description" "Generic connector, single row, 01x04, script generated"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_locked" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "ki_keywords" "connector"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_fp_filters" "Connector*:*_1x??_*"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(symbol "Conn_01x04_Socket_1_1"
				(arc
					(start 0 -4.572)
					(mid -0.5058 -5.08)
					(end 0 -5.588)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(arc
					(start 0 -2.032)
					(mid -0.5058 -2.54)
					(end 0 -3.048)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy -1.27 -5.08) (xy -0.508 -5.08)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy -1.27 -2.54) (xy -0.508 -2.54)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy -1.27 0) (xy -0.508 0)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy -1.27 2.54) (xy -0.508 2.54)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(arc
					(start 0 0.508)
					(mid -0.5058 0)
					(end 0 -0.508)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(arc
					(start 0 3.048)
					(mid -0.5058 2.54)
					(end 0 2.032)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(pin passive line
					(at -5.08 2.54 0)
					(length 3.81)
					(name "VCC"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "1"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at -5.08 0 0)
					(length 3.81)
					(name "GND"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "2"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at -5.08 -2.54 0)
					(length 3.81)
					(name "D0"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "3"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at -5.08 -5.08 0)
					(length 3.81)
					(name "A0"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "4"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
			)
		)
		(symbol "Connector:Conn_01x06_Socket"
			(pin_names
				(offset 1.016) hide)
			(exclude_from_sim no)
			(in_bom yes)
			(on_board yes)
			(property "Reference" "J4"
				(at 5.08 -1.27 90)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "Value" "Modulo bluetooth"
				(at 2.54 -1.27 90)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "Footprint" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Datasheet" "~"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Description" "Generic connector, single row, 01x06, script generated"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_locked" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "ki_keywords" "connector"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_fp_filters" "Connector*:*_1x??_*"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(symbol "Conn_01x06_Socket_1_1"
				(arc
					(start 0 -7.112)
					(mid -0.5058 -7.62)
					(end 0 -8.128)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(arc
					(start 0 -4.572)
					(mid -0.5058 -5.08)
					(end 0 -5.588)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(arc
					(start 0 -2.032)
					(mid -0.5058 -2.54)
					(end 0 -3.048)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy -1.27 -7.62) (xy -0.508 -7.62)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy -1.27 -5.08) (xy -0.508 -5.08)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy -1.27 -2.54) (xy -0.508 -2.54)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy -1.27 0) (xy -0.508 0)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy -1.27 2.54) (xy -0.508 2.54)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy -1.27 5.08) (xy -0.508 5.08)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(arc
					(start 0 0.508)
					(mid -0.5058 0)
					(end 0 -0.508)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(arc
					(start 0 3.048)
					(mid -0.5058 2.54)
					(end 0 2.032)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(arc
					(start 0 5.588)
					(mid -0.5058 5.08)
					(end 0 4.572)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(pin passive line
					(at -5.08 5.08 0)
					(length 3.81)
					(name "State"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "1"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at -5.08 2.54 0)
					(length 3.81)
					(name "RX"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "2"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at -5.08 0 0)
					(length 3.81)
					(name "TX"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "3"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at -5.08 -2.54 0)
					(length 3.81)
					(name "GND"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "4"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at -5.08 -5.08 0)
					(length 3.81)
					(name "VCC"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "5"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at -5.08 -7.62 0)
					(length 3.81)
					(name "EN"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "6"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
			)
		)
		(symbol "Connector:Conn_01x11_Pin"
			(pin_names
				(offset 1.016) hide)
			(exclude_from_sim no)
			(in_bom yes)
			(on_board yes)
			(property "Reference" "J"
				(at 0 15.24 0)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "Value" "Conn_01x11_Pin"
				(at 0 -15.24 0)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "Footprint" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Datasheet" "~"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Description" "Generic connector, single row, 01x11, script generated"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_locked" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "ki_keywords" "connector"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_fp_filters" "Connector*:*_1x??_*"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(symbol "Conn_01x11_Pin_1_1"
				(polyline
					(pts
						(xy 1.27 -12.7) (xy 0.8636 -12.7)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy 1.27 -10.16) (xy 0.8636 -10.16)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy 1.27 -7.62) (xy 0.8636 -7.62)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy 1.27 -5.08) (xy 0.8636 -5.08)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy 1.27 -2.54) (xy 0.8636 -2.54)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy 1.27 0) (xy 0.8636 0)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy 1.27 2.54) (xy 0.8636 2.54)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy 1.27 5.08) (xy 0.8636 5.08)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy 1.27 7.62) (xy 0.8636 7.62)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy 1.27 10.16) (xy 0.8636 10.16)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy 1.27 12.7) (xy 0.8636 12.7)
					)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(rectangle
					(start 0.8636 -12.573)
					(end 0 -12.827)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type outline)
					)
				)
				(rectangle
					(start 0.8636 -10.033)
					(end 0 -10.287)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type outline)
					)
				)
				(rectangle
					(start 0.8636 -7.493)
					(end 0 -7.747)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type outline)
					)
				)
				(rectangle
					(start 0.8636 -4.953)
					(end 0 -5.207)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type outline)
					)
				)
				(rectangle
					(start 0.8636 -2.413)
					(end 0 -2.667)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type outline)
					)
				)
				(rectangle
					(start 0.8636 0.127)
					(end 0 -0.127)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type outline)
					)
				)
				(rectangle
					(start 0.8636 2.667)
					(end 0 2.413)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type outline)
					)
				)
				(rectangle
					(start 0.8636 5.207)
					(end 0 4.953)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type outline)
					)
				)
				(rectangle
					(start 0.8636 7.747)
					(end 0 7.493)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type outline)
					)
				)
				(rectangle
					(start 0.8636 10.287)
					(end 0 10.033)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type outline)
					)
				)
				(rectangle
					(start 0.8636 12.827)
					(end 0 12.573)
					(stroke
						(width 0.1524)
						(type default)
					)
					(fill
						(type outline)
					)
				)
				(pin passive line
					(at 5.08 12.7 180)
					(length 3.81)
					(name "Pin_1"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "1"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at 5.08 -10.16 180)
					(length 3.81)
					(name "Pin_10"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "10"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at 5.08 -12.7 180)
					(length 3.81)
					(name "Pin_11"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "11"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at 5.08 10.16 180)
					(length 3.81)
					(name "Pin_2"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "2"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at 5.08 7.62 180)
					(length 3.81)
					(name "Pin_3"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "3"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at 5.08 5.08 180)
					(length 3.81)
					(name "Pin_4"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "4"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at 5.08 2.54 180)
					(length 3.81)
					(name "Pin_5"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "5"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at 5.08 0 180)
					(length 3.81)
					(name "Pin_6"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "6"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at 5.08 -2.54 180)
					(length 3.81)
					(name "Pin_7"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "7"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at 5.08 -5.08 180)
					(length 3.81)
					(name "Pin_8"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "8"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin passive line
					(at 5.08 -7.62 180)
					(length 3.81)
					(name "Pin_9"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "9"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
			)
		)
		(symbol "MCU_Module:Arduino_Nano_v2.x"
			(exclude_from_sim no)
			(in_bom yes)
			(on_board yes)
			(property "Reference" "A"
				(at -10.16 23.495 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify left bottom)
				)
			)
			(property "Value" "Arduino_Nano_v2.x"
				(at 5.08 -24.13 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify left top)
				)
			)
			(property "Footprint" "Module:Arduino_Nano"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
						(italic yes)
					)
					(hide yes)
				)
			)
			(property "Datasheet" "https://www.arduino.cc/en/uploads/Main/ArduinoNanoManual23.pdf"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Description" "Arduino Nano v2.x"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_keywords" "Arduino nano microcontroller module USB"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_fp_filters" "Arduino*Nano*"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(symbol "Arduino_Nano_v2.x_0_1"
				(rectangle
					(start -10.16 22.86)
					(end 10.16 -22.86)
					(stroke
						(width 0.254)
						(type default)
					)
					(fill
						(type background)
					)
				)
			)
			(symbol "Arduino_Nano_v2.x_1_1"
				(pin bidirectional line
					(at -12.7 12.7 0)
					(length 2.54)
					(name "D1/TX"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "1"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at -12.7 -2.54 0)
					(length 2.54)
					(name "D7"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "10"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at -12.7 -5.08 0)
					(length 2.54)
					(name "D8"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "11"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at -12.7 -7.62 0)
					(length 2.54)
					(name "D9"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "12"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at -12.7 -10.16 0)
					(length 2.54)
					(name "D10"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "13"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at -12.7 -12.7 0)
					(length 2.54)
					(name "D11"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "14"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at -12.7 -15.24 0)
					(length 2.54)
					(name "D12"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "15"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at -12.7 -17.78 0)
					(length 2.54)
					(name "D13"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "16"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin power_out line
					(at 2.54 25.4 270)
					(length 2.54)
					(name "3V3"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "17"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin input line
					(at 12.7 5.08 180)
					(length 2.54)
					(name "AREF"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "18"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at 12.7 0 180)
					(length 2.54)
					(name "A0"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "19"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at -12.7 15.24 0)
					(length 2.54)
					(name "D0/RX"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "2"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at 12.7 -2.54 180)
					(length 2.54)
					(name "A1"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "20"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at 12.7 -5.08 180)
					(length 2.54)
					(name "A2"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "21"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at 12.7 -7.62 180)
					(length 2.54)
					(name "A3"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "22"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at 12.7 -10.16 180)
					(length 2.54)
					(name "A4"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "23"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at 12.7 -12.7 180)
					(length 2.54)
					(name "A5"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "24"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at 12.7 -15.24 180)
					(length 2.54)
					(name "A6"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "25"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at 12.7 -17.78 180)
					(length 2.54)
					(name "A7"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "26"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin power_out line
					(at 5.08 25.4 270)
					(length 2.54)
					(name "+5V"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "27"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin input line
					(at 12.7 15.24 180)
					(length 2.54)
					(name "~{RESET}"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "28"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin power_in line
					(at 2.54 -25.4 90)
					(length 2.54)
					(name "GND"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "29"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin input line
					(at 12.7 12.7 180)
					(length 2.54)
					(name "~{RESET}"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "3"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin power_in line
					(at -2.54 25.4 270)
					(length 2.54)
					(name "VIN"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "30"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin power_in line
					(at 0 -25.4 90)
					(length 2.54)
					(name "GND"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "4"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at -12.7 10.16 0)
					(length 2.54)
					(name "D2"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "5"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at -12.7 7.62 0)
					(length 2.54)
					(name "D3"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "6"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at -12.7 5.08 0)
					(length 2.54)
					(name "D4"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "7"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at -12.7 2.54 0)
					(length 2.54)
					(name "D5"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "8"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin bidirectional line
					(at -12.7 0 0)
					(length 2.54)
					(name "D6"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "9"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
			)
		)
		(symbol "ROB-Board-14450:ROB-14450"
			(pin_names
				(offset 1.016)
			)
			(exclude_from_sim no)
			(in_bom yes)
			(on_board yes)
			(property "Reference" "U"
				(at -10.16 16.002 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify left bottom)
				)
			)
			(property "Value" "ROB-14450"
				(at -10.16 -20.32 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify left bottom)
				)
			)
			(property "Footprint" "ROB-14450:MODULE_ROB-14450"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
					(hide yes)
				)
			)
			(property "Datasheet" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Description" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "PARTREV" "11-13-17"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
					(hide yes)
				)
			)
			(property "MANUFACTURER" "Sparkfun Electronics"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
					(hide yes)
				)
			)
			(property "STANDARD" "Manufacturer Recommendation"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
					(hide yes)
				)
			)
			(symbol "ROB-14450_0_0"
				(rectangle
					(start -10.16 -17.78)
					(end 10.16 15.24)
					(stroke
						(width 0.254)
						(type default)
					)
					(fill
						(type background)
					)
				)
				(pin power_in line
					(at 15.24 10.16 180)
					(length 5.08)
					(name "VM"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "1"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin input line
					(at -15.24 10.16 0)
					(length 5.08)
					(name "PWMB"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "10"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin input line
					(at -15.24 -5.08 0)
					(length 5.08)
					(name "BI2"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "11"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin input line
					(at -15.24 -2.54 0)
					(length 5.08)
					(name "BI1"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "12"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin input line
					(at -15.24 -10.16 0)
					(length 5.08)
					(name "STBY"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "13"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin input line
					(at -15.24 5.08 0)
					(length 5.08)
					(name "AI1"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "14"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin input line
					(at -15.24 2.54 0)
					(length 5.08)
					(name "AI2"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "15"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin input line
					(at -15.24 12.7 0)
					(length 5.08)
					(name "PWMA"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "16"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin power_in line
					(at 15.24 12.7 180)
					(length 5.08)
					(name "VCC"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "2"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin power_in line
					(at 15.24 -10.16 180)
					(length 5.08)
					(name "GND"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "3"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin output line
					(at 15.24 5.08 180)
					(length 5.08)
					(name "A01"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "4"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin output line
					(at 15.24 2.54 180)
					(length 5.08)
					(name "A02"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "5"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin output line
					(at 15.24 -5.08 180)
					(length 5.08)
					(name "B02"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "6"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin output line
					(at 15.24 -2.54 180)
					(length 5.08)
					(name "B01"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "7"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin power_in line
					(at 15.24 -12.7 180)
					(length 5.08)
					(name "GND__1"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "8"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin power_in line
					(at 15.24 -15.24 180)
					(length 5.08)
					(name "GND__2"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "9"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
			)
		)
		(symbol "Regulator_Linear:LM7805_TO220"
			(pin_names
				(offset 0.254)
			)
			(exclude_from_sim no)
			(in_bom yes)
			(on_board yes)
			(property "Reference" "U"
				(at -3.81 3.175 0)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "Value" "LM7805_TO220"
				(at 0 3.175 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify left)
				)
			)
			(property "Footprint" "Package_TO_SOT_THT:TO-220-3_Vertical"
				(at 0 5.715 0)
				(effects
					(font
						(size 1.27 1.27)
						(italic yes)
					)
					(hide yes)
				)
			)
			(property "Datasheet" "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF"
				(at 0 -1.27 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Description" "Positive 1A 35V Linear Regulator, Fixed Output 5V, TO-220"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_keywords" "Voltage Regulator 1A Positive"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_fp_filters" "TO?220*"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(symbol "LM7805_TO220_0_1"
				(rectangle
					(start -5.08 1.905)
					(end 5.08 -5.08)
					(stroke
						(width 0.254)
						(type default)
					)
					(fill
						(type background)
					)
				)
			)
			(symbol "LM7805_TO220_1_1"
				(pin power_in line
					(at -7.62 0 0)
					(length 2.54)
					(name "VI"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "1"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin power_in line
					(at 0 -7.62 90)
					(length 2.54)
					(name "GND"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "2"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
				(pin power_out line
					(at 7.62 0 180)
					(length 2.54)
					(name "VO"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "3"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
			)
		)
		(symbol "Switch EG1213:EG1213"
			(pin_names
				(offset 1.016)
			)
			(exclude_from_sim no)
			(in_bom yes)
			(on_board yes)
			(property "Reference" "S"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
				)
			)
			(property "Value" "EG1213"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
				)
			)
			(property "Footprint" "EG1213:SW_EG1213"
				(at -1.016 -9.144 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
					(hide yes)
				)
			)
			(property "Datasheet" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Description" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "MF" "E-Switch"
				(at -0.254 -5.08 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
					(hide yes)
				)
			)
			(property "Description_1" "\nSlide Switch, EG Series, SPDT, Non-Shorting, ON-ON, 200mA DC, 30VDC, R/A, PC | E-Switch EG1213\n"
				(at 4.826 -22.098 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
					(hide yes)
				)
			)
			(property "Package" "None"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
					(hide yes)
				)
			)
			(property "Price" "None"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
					(hide yes)
				)
			)
			(property "Check_prices" "https://www.snapeda.com/parts/EG1213/E-Switch/view-part/?ref=eda"
				(at 0.254 12.954 0)
				(do_not_autoplace)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
					(hide yes)
				)
			)
			(property "SnapEDA_Link" "https://www.snapeda.com/parts/EG1213/E-Switch/view-part/?ref=snap"
				(at 1.016 -17.018 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
					(hide yes)
				)
			)
			(property "MP" "EG1213"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
					(hide yes)
				)
			)
			(property "Purchase-URL" "https://www.snapeda.com/api/url_track_click_mouser/?unipart_id=13820&manufacturer=E-Switch&part_name=EG1213&search_term=None"
				(at -1.27 -39.878 0)
				(do_not_autoplace)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
					(hide yes)
				)
			)
			(property "Availability" "In Stock"
				(at 14.986 1.016 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
					(hide yes)
				)
			)
			(property "MANUFACTURER" "E-Switch"
				(at 14.478 -4.572 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(justify bottom)
					(hide yes)
				)
			)
			(symbol "EG1213_0_0"
				(circle
					(center -1.778 -2.54)
					(radius 0.568)
					(stroke
						(width 0.254)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(circle
					(center -1.778 2.54)
					(radius 0.568)
					(stroke
						(width 0.254)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(polyline
					(pts
						(xy 1.016 0) (xy -2.286 2.032)
					)
					(stroke
						(width 0.254)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(circle
					(center 1.778 0)
					(radius 0.568)
					(stroke
						(width 0.254)
						(type default)
					)
					(fill
						(type none)
					)
				)
				(pin passive line
					(at -7.62 2.54 0)
					(length 5.08)
					(name "~"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "1"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin passive line
					(at 7.62 0 180)
					(length 5.08)
					(name "~"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "2"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
				(pin passive line
					(at -7.62 -2.54 0)
					(length 5.08)
					(name "~"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
					(number "3"
						(effects
							(font
								(size 1.016 1.016)
							)
						)
					)
				)
			)
		)
		(symbol "power:GND"
			(power)
			(pin_numbers hide)
			(pin_names
				(offset 0) hide)
			(exclude_from_sim no)
			(in_bom yes)
			(on_board yes)
			(property "Reference" "#PWR"
				(at 0 -6.35 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Value" "GND"
				(at 0 -3.81 0)
				(effects
					(font
						(size 1.27 1.27)
					)
				)
			)
			(property "Footprint" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Datasheet" ""
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "Description" "Power symbol creates a global label with name \"GND\" , ground"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(property "ki_keywords" "global power"
				(at 0 0 0)
				(effects
					(font
						(size 1.27 1.27)
					)
					(hide yes)
				)
			)
			(symbol "GND_0_1"
				(polyline
					(pts
						(xy 0 0) (xy 0 -1.27) (xy 1.27 -1.27) (xy 0 -2.54) (xy -1.27 -1.27) (xy 0 -1.27)
					)
					(stroke
						(width 0)
						(type default)
					)
					(fill
						(type none)
					)
				)
			)
			(symbol "GND_1_1"
				(pin power_in line
					(at 0 0 270)
					(length 0)
					(name "~"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
					(number "1"
						(effects
							(font
								(size 1.27 1.27)
							)
						)
					)
				)
			)
		)
	)
	(junction
		(at 59.69 140.97)
		(diameter 0)
		(color 0 0 0 0)
		(uuid "06f700c0-c14d-457d-9056-bd70d488e3f2")
	)
	(junction
		(at 82.55 88.9)
		(diameter 0)
		(color 0 0 0 0)
		(uuid "3227471a-e8f4-4712-be42-adf1d32e6df4")
	)
	(junction
		(at 66.04 78.74)
		(diameter 0)
		(color 0 0 0 0)
		(uuid "60fb6165-f4de-46e1-950f-af65d36c95be")
	)
	(junction
		(at 52.07 140.97)
		(diameter 0)
		(color 0 0 0 0)
		(uuid "a06a3bde-51e5-4fbf-a304-f9f41fcc2e73")
	)
	(junction
		(at 170.18 93.98)
		(diameter 0)
		(color 0 0 0 0)
		(uuid "a2544f4a-76ef-4faf-a6b8-b5b0bbe2e2f8")
	)
	(junction
		(at 60.96 78.74)
		(diameter 0)
		(color 0 0 0 0)
		(uuid "b9939435-32c6-4bdc-9a17-19018312f111")
	)
	(junction
		(at 52.07 77.47)
		(diameter 0)
		(color 0 0 0 0)
		(uuid "d344d562-fa71-4049-ade8-52043ad65470")
	)
	(junction
		(at 60.96 81.28)
		(diameter 0)
		(color 0 0 0 0)
		(uuid "deb4b4de-65f5-43f1-ab23-785fd1af94f6")
	)
	(junction
		(at 87.63 86.36)
		(diameter 0)
		(color 0 0 0 0)
		(uuid "ea10c059-ed30-4df7-abc3-5b190e4172e5")
	)
	(wire
		(pts
			(xy 52.07 78.74) (xy 52.07 77.47)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "02c57a48-4d9b-4c76-9a81-3926ed0635a7")
	)
	(wire
		(pts
			(xy 114.3 137.16) (xy 118.11 137.16)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "02dc690f-ddd3-47bc-8a17-4a0269d09913")
	)
	(wire
		(pts
			(xy 49.53 140.97) (xy 52.07 140.97)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "0e2e911e-fd4d-44ba-8789-bab293047cc8")
	)
	(wire
		(pts
			(xy 82.55 115.57) (xy 91.44 115.57)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "1020b4c7-1734-4d6b-875b-b1eccd65ea8d")
	)
	(wire
		(pts
			(xy 80.01 88.9) (xy 80.01 91.44)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "1056bcc5-1a68-4347-b640-900b7c723c9f")
	)
	(wire
		(pts
			(xy 110.49 96.52) (xy 110.49 91.44)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "10d797fc-7b20-4194-a3c2-bf845512ad9d")
	)
	(wire
		(pts
			(xy 182.88 99.06) (xy 186.69 99.06)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "123acd8a-86ec-4313-a55c-06eb5149c956")
	)
	(wire
		(pts
			(xy 77.47 111.76) (xy 82.55 111.76)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "1247dc2b-5fc7-4a2c-b27a-774dbbbd9210")
	)
	(wire
		(pts
			(xy 78.74 124.46) (xy 78.74 116.84)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "13a8c356-55fd-49c9-9636-8eda5c2c9924")
	)
	(wire
		(pts
			(xy 52.07 93.98) (xy 38.1 93.98)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "15501910-7ce1-4f82-84c8-75035cb6c60e")
	)
	(wire
		(pts
			(xy 66.04 81.28) (xy 64.77 81.28)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "1586d671-3b4c-4758-8249-e82fca961fdd")
	)
	(wire
		(pts
			(xy 83.82 91.44) (xy 83.82 93.98)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "15c7ed13-cdba-404d-83d5-f376229a92b1")
	)
	(wire
		(pts
			(xy 86.36 114.3) (xy 86.36 104.14)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "1aeab7d6-5858-4d16-86f2-4d92be23e10d")
	)
	(wire
		(pts
			(xy 60.96 78.74) (xy 52.07 78.74)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "1dcfee01-c84a-48a2-a4ce-494d1a4e0366")
	)
	(wire
		(pts
			(xy 38.1 90.17) (xy 36.83 90.17)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "23c41e31-68a5-4aa3-9765-693b91498b09")
	)
	(wire
		(pts
			(xy 38.1 91.44) (xy 38.1 90.17)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "2563c211-acbd-4be9-af94-610801e19b80")
	)
	(wire
		(pts
			(xy 80.01 116.84) (xy 80.01 109.22)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "26f059d4-273c-4ab8-9250-cb521f45d655")
	)
	(wire
		(pts
			(xy 36.83 90.17) (xy 36.83 93.98)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "27977e6e-8710-4ebc-946b-1f5eb65aef45")
	)
	(wire
		(pts
			(xy 36.83 93.98) (xy 34.29 93.98)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "299561c8-7f49-4730-8fb7-674ff4d5cc29")
	)
	(wire
		(pts
			(xy 82.55 88.9) (xy 97.79 88.9)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "29b96d1e-4e87-477b-9c7b-3a4f1c1a32cc")
	)
	(wire
		(pts
			(xy 186.69 104.14) (xy 186.69 101.6)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "29c504c6-8231-4e0c-b169-01b410280222")
	)
	(wire
		(pts
			(xy 38.1 96.52) (xy 34.29 96.52)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "2cafba5f-4c84-4794-81fd-1a5bee7658c9")
	)
	(wire
		(pts
			(xy 163.83 109.22) (xy 189.23 109.22)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "2d78a44e-7609-455b-a1b9-d28fee5c05db")
	)
	(wire
		(pts
			(xy 64.77 144.78) (xy 59.69 144.78)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "2dd49bba-ab14-401d-a9fc-204e42e688e0")
	)
	(wire
		(pts
			(xy 104.14 124.46) (xy 78.74 124.46)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "31701aba-4b52-417b-9c70-72ea4af208b6")
	)
	(wire
		(pts
			(xy 110.49 91.44) (xy 83.82 91.44)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "358986a6-f463-4930-bbd6-3458c038568d")
	)
	(wire
		(pts
			(xy 41.91 104.14) (xy 34.29 104.14)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "367260ac-4fa1-4d0f-be2e-814898035a2e")
	)
	(wire
		(pts
			(xy 88.9 101.6) (xy 77.47 101.6)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "367fcbe3-0e3f-49bb-a30d-5cfcb940b32d")
	)
	(wire
		(pts
			(xy 170.18 93.98) (xy 170.18 96.52)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "36c00a14-4a57-4652-8918-61acd423b69e")
	)
	(wire
		(pts
			(xy 77.47 119.38) (xy 123.19 119.38)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "38513153-4d97-4327-adbf-069b6a5886b8")
	)
	(wire
		(pts
			(xy 52.07 77.47) (xy 50.8 77.47)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "38ba90d3-892f-4fef-97d2-95bf3df1771f")
	)
	(wire
		(pts
			(xy 133.35 93.98) (xy 104.14 93.98)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "3d5c6eea-2c01-4714-a26a-bd7e0dbd5bb8")
	)
	(wire
		(pts
			(xy 185.42 111.76) (xy 185.42 116.84)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "3e99e007-a564-4d2b-a5c5-26e72903e5fc")
	)
	(wire
		(pts
			(xy 182.88 101.6) (xy 182.88 99.06)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "4053cb25-af85-4838-a194-068737c19036")
	)
	(wire
		(pts
			(xy 72.39 62.23) (xy 72.39 59.69)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "40fc4c12-1b58-464b-985c-207637a927a1")
	)
	(wire
		(pts
			(xy 163.83 111.76) (xy 185.42 111.76)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "43968405-c304-41c7-99c3-eca7682adf33")
	)
	(wire
		(pts
			(xy 163.83 96.52) (xy 170.18 96.52)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "44064cb4-adda-41d3-8451-546212ecb404")
	)
	(wire
		(pts
			(xy 40.64 101.6) (xy 34.29 101.6)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "46c7d9ac-5e85-45dc-8440-4d5ccf8892cc")
	)
	(wire
		(pts
			(xy 120.65 134.62) (xy 120.65 140.97)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "49a816ca-1e25-4145-8762-c5e33f3730c1")
	)
	(wire
		(pts
			(xy 52.07 99.06) (xy 40.64 99.06)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "4bf76cff-2496-4b42-bf74-7fa0b9c6c0fd")
	)
	(wire
		(pts
			(xy 59.69 140.97) (xy 52.07 140.97)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "4c374bff-f71f-4db4-b537-dcf6fc33c5de")
	)
	(wire
		(pts
			(xy 77.47 86.36) (xy 87.63 86.36)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "4f9f0dbb-b600-4849-9c6e-a25a22948b8e")
	)
	(wire
		(pts
			(xy 35.56 88.9) (xy 35.56 91.44)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "50b36a0f-c88d-4d43-8465-c82bbf12b743")
	)
	(wire
		(pts
			(xy 80.01 109.22) (xy 77.47 109.22)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "5255ecfb-2b56-41cf-8d20-f10c1a44a50e")
	)
	(wire
		(pts
			(xy 60.96 60.96) (xy 60.96 59.69)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "52dd3f20-3999-4ed8-b867-d9769dfa511c")
	)
	(wire
		(pts
			(xy 189.23 73.66) (xy 189.23 71.12)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "5319a79d-c02f-4230-902d-2ac16d90ac39")
	)
	(wire
		(pts
			(xy 191.77 71.12) (xy 191.77 78.74)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "55cc0f55-c7a9-4762-9654-fd2ed8b79ea9")
	)
	(wire
		(pts
			(xy 52.07 88.9) (xy 35.56 88.9)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "5a75d2d2-91a4-49b8-9bac-35525253c869")
	)
	(wire
		(pts
			(xy 80.01 91.44) (xy 77.47 91.44)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "5a97f4c0-8202-4c3d-bb74-f79b99e0c886")
	)
	(wire
		(pts
			(xy 90.17 69.85) (xy 90.17 68.58)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "5b19e45e-d52b-421c-8426-36401eb273ce")
	)
	(wire
		(pts
			(xy 60.96 81.28) (xy 62.23 81.28)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "5f92f98e-ae6c-4a43-9818-1ec0e724b38b")
	)
	(wire
		(pts
			(xy 52.07 91.44) (xy 38.1 91.44)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "61ff0c79-0c27-45b8-a777-e5442707714b")
	)
	(wire
		(pts
			(xy 133.35 109.22) (xy 88.9 109.22)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "632ac119-1d01-4d33-b543-60ad75195ac0")
	)
	(wire
		(pts
			(xy 118.11 140.97) (xy 118.11 137.16)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "64ad0df6-e6ae-4991-bb80-32348c8bb04a")
	)
	(wire
		(pts
			(xy 189.23 109.22) (xy 189.23 114.3)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "66e0bf63-bd0e-4873-925a-dc882b34caa9")
	)
	(wire
		(pts
			(xy 78.74 116.84) (xy 77.47 116.84)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "686ddb2f-5594-4994-b491-db6525dec1ee")
	)
	(wire
		(pts
			(xy 165.1 93.98) (xy 163.83 93.98)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "6884bd12-1945-4a24-bbec-bba0e0ce80fd")
	)
	(wire
		(pts
			(xy 38.1 93.98) (xy 38.1 96.52)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "6b2f2a8a-dc6b-4358-bf1b-530cc8144d78")
	)
	(wire
		(pts
			(xy 50.8 77.47) (xy 50.8 74.93)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "6d88d75f-0e8d-4abc-9f09-4bf0ba490553")
	)
	(wire
		(pts
			(xy 52.07 77.47) (xy 53.34 77.47)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "6dcf4dce-7e88-4678-bf2a-d69ea9788608")
	)
	(wire
		(pts
			(xy 52.07 140.97) (xy 52.07 143.51)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "6e624194-04f9-4e04-9edc-2984d437c14f")
	)
	(wire
		(pts
			(xy 62.23 60.96) (xy 62.23 62.23)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "6ea1972f-12ed-450d-b6f4-fdfe12c03be5")
	)
	(wire
		(pts
			(xy 77.47 88.9) (xy 77.47 86.36)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "70f32abe-7f67-42db-9234-0d87ff25ab83")
	)
	(wire
		(pts
			(xy 39.37 96.52) (xy 39.37 99.06)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "7bca58b9-167c-434e-85e7-73f7cba3d76a")
	)
	(wire
		(pts
			(xy 77.47 85.09) (xy 87.63 85.09)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "822b2df0-66ac-433c-8d11-902ae49abf3f")
	)
	(wire
		(pts
			(xy 59.69 140.97) (xy 59.69 144.78)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "86550b24-bf66-4d74-b136-359eb7cbf180")
	)
	(wire
		(pts
			(xy 62.23 62.23) (xy 72.39 62.23)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "88eb9c8f-7357-473a-baec-93ec75682f79")
	)
	(wire
		(pts
			(xy 77.47 114.3) (xy 86.36 114.3)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "8926ad19-4bfd-4b07-95d8-8d3d45827e39")
	)
	(wire
		(pts
			(xy 177.8 63.5) (xy 177.8 72.39)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "8c3ee705-6abc-44e8-918a-b4b065111d74")
	)
	(wire
		(pts
			(xy 60.96 78.74) (xy 60.96 81.28)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "8e4d3f41-1f5b-412a-950f-9fef45c119db")
	)
	(wire
		(pts
			(xy 133.35 111.76) (xy 85.09 111.76)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "8e9ab797-1764-463f-b88b-e94d6b71c5d2")
	)
	(wire
		(pts
			(xy 39.37 99.06) (xy 34.29 99.06)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "8fb53d33-5ea2-4c0f-8975-aab5af93f371")
	)
	(wire
		(pts
			(xy 91.44 115.57) (xy 91.44 101.6)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "9390609c-0ae2-4bae-ae93-cff8628bcbd6")
	)
	(wire
		(pts
			(xy 53.34 77.47) (xy 53.34 74.93)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "9612acb4-6f3b-4a44-9c4b-cdf235e04bfb")
	)
	(wire
		(pts
			(xy 52.07 104.14) (xy 43.18 104.14)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "974c0896-39fd-4603-bdd9-20e4f7d77e6b")
	)
	(wire
		(pts
			(xy 123.19 119.38) (xy 123.19 140.97)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "9f3e800d-6faa-4033-b2b6-2d64faaf3e67")
	)
	(wire
		(pts
			(xy 41.91 101.6) (xy 41.91 104.14)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "a1c109ec-97c9-45b1-a817-ad822d980e39")
	)
	(wire
		(pts
			(xy 80.01 88.9) (xy 82.55 88.9)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "a1f4ef70-39a6-4536-8600-adb6477d8663")
	)
	(wire
		(pts
			(xy 77.47 121.92) (xy 125.73 121.92)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "a6d97386-21d1-451d-80e9-190a11b5a20e")
	)
	(wire
		(pts
			(xy 66.04 78.74) (xy 66.04 81.28)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "ae2cbf93-a746-4912-9894-25e394ecd0cd")
	)
	(wire
		(pts
			(xy 52.07 106.68) (xy 44.45 106.68)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "b361ba99-8ae0-4283-98bb-dc75b71d9fcc")
	)
	(wire
		(pts
			(xy 86.36 104.14) (xy 133.35 104.14)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "b36db93a-7feb-4738-a266-9b9e1f48c6f6")
	)
	(wire
		(pts
			(xy 43.18 104.14) (xy 43.18 106.68)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "b4c021c5-42a0-4477-8d55-89654d3b5f2f")
	)
	(wire
		(pts
			(xy 88.9 109.22) (xy 88.9 101.6)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "b776b4c8-0a0b-4b54-8b2d-58d7b0b205d1")
	)
	(wire
		(pts
			(xy 59.69 132.08) (xy 59.69 140.97)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "b980976b-e2a2-47ad-b30f-960a9894239d")
	)
	(wire
		(pts
			(xy 52.07 101.6) (xy 41.91 101.6)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "b980a48c-c9f2-4a3f-b856-25348094cb37")
	)
	(wire
		(pts
			(xy 185.42 116.84) (xy 189.23 116.84)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "bbda9d85-4348-457d-b734-c6e4e1d81cc1")
	)
	(wire
		(pts
			(xy 80.01 116.84) (xy 133.35 116.84)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "bc9c8e65-3c0a-4eb0-801b-932834845abb")
	)
	(wire
		(pts
			(xy 90.17 68.58) (xy 92.71 68.58)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "bcdd61d8-512d-4c63-9a82-5e45eefe4503")
	)
	(wire
		(pts
			(xy 133.35 96.52) (xy 110.49 96.52)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "bf9bb6ca-f405-4370-8e9b-b42f527f6e2a")
	)
	(wire
		(pts
			(xy 62.23 60.96) (xy 60.96 60.96)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "bfbbf5e0-787a-499b-aaa8-b08a53156c03")
	)
	(wire
		(pts
			(xy 40.64 99.06) (xy 40.64 101.6)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "c0119a6b-d83d-4329-b7bd-b5e913158db5")
	)
	(wire
		(pts
			(xy 163.83 104.14) (xy 186.69 104.14)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "c26a6c3d-23a2-457d-9fb1-478a15c6f95c")
	)
	(wire
		(pts
			(xy 64.77 146.05) (xy 62.23 146.05)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "c9edcca1-3a90-4e62-9878-1d574d4e169a")
	)
	(wire
		(pts
			(xy 52.07 96.52) (xy 39.37 96.52)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "cd519e0d-5ba4-47fe-8442-74fd760893fe")
	)
	(wire
		(pts
			(xy 77.47 59.69) (xy 77.47 85.09)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "cd80dd93-44e5-4057-9b5b-6d217713cd9e")
	)
	(wire
		(pts
			(xy 163.83 101.6) (xy 182.88 101.6)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "d0c8b8dc-7a7c-4ded-a575-de34fadcff7b")
	)
	(wire
		(pts
			(xy 92.71 68.58) (xy 92.71 67.31)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "d22c50cd-0de2-43c0-820f-d737f69f2c94")
	)
	(wire
		(pts
			(xy 133.35 101.6) (xy 91.44 101.6)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "d3eca55f-45eb-4212-a034-448000901939")
	)
	(wire
		(pts
			(xy 104.14 93.98) (xy 104.14 124.46)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "d989b1a3-a97a-41ab-ab9e-1936f32534a8")
	)
	(wire
		(pts
			(xy 64.77 146.05) (xy 64.77 144.78)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "daa5d7d1-68ca-4b22-8325-c7f32a4cff68")
	)
	(wire
		(pts
			(xy 66.04 78.74) (xy 60.96 78.74)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "dca92406-9b9e-4586-a476-789c59c4a337")
	)
	(wire
		(pts
			(xy 95.25 68.58) (xy 95.25 67.31)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "dfd68967-a6eb-4cf8-99a1-bb44a1cc5520")
	)
	(wire
		(pts
			(xy 125.73 121.92) (xy 125.73 140.97)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "e1a651ce-54d0-4bb2-a15a-6b978bc02b1d")
	)
	(wire
		(pts
			(xy 186.69 77.47) (xy 186.69 78.74)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "e1ca49d7-0468-4e9e-a97d-b4b99b07e936")
	)
	(wire
		(pts
			(xy 43.18 106.68) (xy 34.29 106.68)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "e22aa17f-549d-4267-81ea-53b307b1fa3d")
	)
	(wire
		(pts
			(xy 85.09 99.06) (xy 77.47 99.06)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "e5569f26-cfad-4894-b597-06476fffdbf2")
	)
	(wire
		(pts
			(xy 170.18 93.98) (xy 189.23 93.98)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "e6cf3a96-8ed2-4108-b58b-270ff7aea435")
	)
	(wire
		(pts
			(xy 44.45 106.68) (xy 44.45 109.22)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "e85a0ddd-d8ed-4985-b20b-61510798484a")
	)
	(wire
		(pts
			(xy 62.23 146.05) (xy 62.23 148.59)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "eb04778e-9fb9-4b6b-80e1-ae8fee5fb4ff")
	)
	(wire
		(pts
			(xy 52.07 143.51) (xy 49.53 143.51)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "ec78c9f6-41f0-435d-804e-f48be230101f")
	)
	(wire
		(pts
			(xy 35.56 91.44) (xy 34.29 91.44)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "ecdd6a1c-3e7c-4969-a795-0a48bbe95af5")
	)
	(wire
		(pts
			(xy 170.18 55.88) (xy 170.18 93.98)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "f19a2d9e-9fb2-4d2b-b62d-e511aaa1c4e8")
	)
	(wire
		(pts
			(xy 85.09 111.76) (xy 85.09 99.06)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "f66d769e-4909-4ab7-97e9-5a912a48028b")
	)
	(wire
		(pts
			(xy 83.82 93.98) (xy 77.47 93.98)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "f8b77170-38de-4ac2-bf46-ea70a0a6a4c4")
	)
	(wire
		(pts
			(xy 44.45 109.22) (xy 34.29 109.22)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "f92d4e9b-946e-47c1-a298-41f16db40c9a")
	)
	(wire
		(pts
			(xy 82.55 111.76) (xy 82.55 115.57)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "f98fcdfa-0ef3-4814-9faa-ef94ca0d888d")
	)
	(wire
		(pts
			(xy 60.96 81.28) (xy 59.69 81.28)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "fe313b8f-7f15-413c-83cc-ff52450b8fbf")
	)
	(wire
		(pts
			(xy 97.79 67.31) (xy 97.79 88.9)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "feeeecbc-80c1-44b4-9068-437132e0333f")
	)
	(wire
		(pts
			(xy 87.63 85.09) (xy 87.63 86.36)
		)
		(stroke
			(width 0)
			(type default)
		)
		(uuid "ff2929ee-db61-46cb-8ab0-eb95ed527c5a")
	)
	(label "Alimentação ponte H"
		(at 165.1 93.98 90)
		(fields_autoplaced yes)
		(effects
			(font
				(size 1.27 1.27)
			)
			(justify left bottom)
		)
		(uuid "5ae91392-0573-40cc-8312-091051806e4e")
	)
	(label "Alimentação sensor lateral"
		(at 62.23 148.59 270)
		(fields_autoplaced yes)
		(effects
			(font
				(size 1.27 1.27)
			)
			(justify right bottom)
		)
		(uuid "81aee80f-e0c7-40b1-979c-5ecdf2f72dd2")
	)
	(label "Alimentação Modulo bluetooth"
		(at 64.77 144.78 270)
		(fields_autoplaced yes)
		(effects
			(font
				(size 1.27 1.27)
			)
			(justify right bottom)
		)
		(uuid "8e1b32cc-5363-412d-873a-8ce5908638ea")
	)
	(label "Alimentação Modulo bluetooth"
		(at 114.3 137.16 180)
		(fields_autoplaced yes)
		(effects
			(font
				(size 1.27 1.27)
			)
			(justify right bottom)
		)
		(uuid "a44132db-65bf-4c28-9faa-6bdc05846bf0")
	)
	(label "Alimentação ponte H"
		(at 59.69 132.08 270)
		(fields_autoplaced yes)
		(effects
			(font
				(size 1.27 1.27)
			)
			(justify right bottom)
		)
		(uuid "e09c9e0c-10ac-455e-9bd9-d1af857a22ad")
	)
	(label "Alimentação sensor lateral"
		(at 90.17 69.85 90)
		(fields_autoplaced yes)
		(effects
			(font
				(size 1.27 1.27)
			)
			(justify left bottom)
		)
		(uuid "f88238f4-3caf-4f1c-9f05-14fcbf2740a8")
	)
	(label "Alimentação sensor lateral"
		(at 60.96 59.69 180)
		(fields_autoplaced yes)
		(effects
			(font
				(size 1.27 1.27)
			)
			(justify right bottom)
		)
		(uuid "ff03dfd0-4149-4b90-9929-8e7baaff5006")
	)
	(global_label "Bateria Arduino"
		(shape input)
		(at 67.31 132.08 270)
		(fields_autoplaced yes)
		(effects
			(font
				(size 1.27 1.27)
			)
			(justify right)
		)
		(uuid "6153551c-e7de-41f3-8dd9-98d7b0a70468")
		(property "Intersheetrefs" "${INTERSHEET_REFS}"
			(at 67.31 149.8817 90)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify right)
				(hide yes)
			)
		)
	)
	(global_label "Bateria Arduino"
		(shape input)
		(at 185.42 55.88 0)
		(fields_autoplaced yes)
		(effects
			(font
				(size 1.27 1.27)
			)
			(justify left)
		)
		(uuid "6c1d9433-5310-4888-b4b3-362c65ac8c87")
		(property "Intersheetrefs" "${INTERSHEET_REFS}"
			(at 203.2217 55.88 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify left)
				(hide yes)
			)
		)
	)
	(global_label "Alimentação sensores"
		(shape input)
		(at 59.69 132.08 180)
		(fields_autoplaced yes)
		(effects
			(font
				(size 1.27 1.27)
			)
			(justify right)
		)
		(uuid "bdf66210-49d3-498a-b4e8-f16759778775")
		(property "Intersheetrefs" "${INTERSHEET_REFS}"
			(at 35.8407 132.08 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify right)
				(hide yes)
			)
		)
	)
	(global_label "Alimentação sensores"
		(shape input)
		(at 34.29 88.9 90)
		(fields_autoplaced yes)
		(effects
			(font
				(size 1.27 1.27)
			)
			(justify left)
		)
		(uuid "c2b9f217-ed7d-4385-8ed6-01ef13d6e02c")
		(property "Intersheetrefs" "${INTERSHEET_REFS}"
			(at 34.29 65.0507 90)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify left)
				(hide yes)
			)
		)
	)
	(symbol
		(lib_id "power:GND")
		(at 186.69 77.47 180)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "1bda0d41-3e5b-4e0b-9b95-c44d9cc52698")
		(property "Reference" "#PWR011"
			(at 186.69 71.12 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Value" "GND"
			(at 186.69 72.39 0)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Footprint" ""
			(at 186.69 77.47 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" ""
			(at 186.69 77.47 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Power symbol creates a global label with name \"GND\" , ground"
			(at 186.69 77.47 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "1"
			(uuid "cc314997-b952-42f3-88f8-84cecc5455c1")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "#PWR011")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "Connector:Conn_01x06_Socket")
		(at 123.19 146.05 270)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(uuid "1e0d3ed5-f1a4-4777-832a-88d6d28af69e")
		(property "Reference" "J4"
			(at 121.92 151.13 90)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Value" "Modulo bluetooth"
			(at 122.936 148.336 90)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Footprint" "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical"
			(at 123.19 146.05 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" "~"
			(at 123.19 146.05 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Generic connector, single row, 01x06, script generated"
			(at 123.19 146.05 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "5"
			(uuid "e55a3ff6-1adb-4d63-97f2-3f83ae1e7897")
		)
		(pin "6"
			(uuid "c706bc8c-09c3-4783-b963-695d42886639")
		)
		(pin "3"
			(uuid "6c50ce5a-7eff-4404-8f98-a79239bcba98")
		)
		(pin "2"
			(uuid "049b8611-50f0-48ec-9502-bf5148b67e8f")
		)
		(pin "1"
			(uuid "49ddac69-961c-4123-b451-d817114de1c4")
		)
		(pin "4"
			(uuid "09dee158-d99d-4833-9fcf-20bd23351fa4")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "J4")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_name "Conn_01x02_Socket_1")
		(lib_id "Connector:Conn_01x02_Socket")
		(at 50.8 69.85 90)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "1e8e5a0d-c317-454c-b146-6170d9ea8535")
		(property "Reference" "J10"
			(at 54.61 69.2149 90)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify right)
			)
		)
		(property "Value" "GND pinos"
			(at 54.61 71.7549 90)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify right)
			)
		)
		(property "Footprint" "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical"
			(at 50.8 69.85 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" "~"
			(at 50.8 69.85 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Generic connector, single row, 01x02, script generated"
			(at 50.8 69.85 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "2"
			(uuid "991d4f6d-247a-41ff-ad3c-d5bedac185c6")
		)
		(pin "1"
			(uuid "181234af-3b0b-4952-b552-810e82a35322")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "J10")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "Connector:Conn_01x04_Socket")
		(at 95.25 62.23 90)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "29b8fefe-d74c-4184-b3f4-6a8a877f46f1")
		(property "Reference" "J6"
			(at 96.52 57.15 90)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Value" "Sensor Lateral - Direita"
			(at 96.52 59.69 90)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Footprint" "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Horizontal"
			(at 95.25 62.23 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" "~"
			(at 95.25 62.23 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Generic connector, single row, 01x04, script generated"
			(at 95.25 62.23 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "1"
			(uuid "0ddae322-5df9-4e2d-8638-7a33d3660a07")
		)
		(pin "2"
			(uuid "f12a7a99-cbc3-4b11-bf0b-6d3f156b855b")
		)
		(pin "3"
			(uuid "4f1e8a9f-22ff-4084-8bde-3aec50ddcaec")
		)
		(pin "4"
			(uuid "ba1c6b51-2ab2-49ca-bd60-09d5ffd22a21")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "J6")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "Connector:Conn_01x01_Socket")
		(at 87.63 88.9 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "3b7fa10e-4bd6-4ee5-9c13-0d9790f390ce")
		(property "Reference" "J8"
			(at 88.9 87.6299 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify left)
			)
		)
		(property "Value" "Conector sensor lateral Direita"
			(at 88.9 90.1699 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify left)
			)
		)
		(property "Footprint" "Connector_PinSocket_2.54mm:PinSocket_1x01_P2.54mm_Vertical"
			(at 87.63 88.9 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" "~"
			(at 87.63 88.9 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Generic connector, single row, 01x01, script generated"
			(at 87.63 88.9 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "1"
			(uuid "7b58f0fb-6f52-4ebc-83d7-a23f32efec45")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "J8")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "power:GND")
		(at 189.23 73.66 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "40d6cd13-7ca2-4a89-8f9e-becebe2024a6")
		(property "Reference" "#PWR09"
			(at 189.23 80.01 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Value" "GND"
			(at 191.77 74.9299 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify left)
			)
		)
		(property "Footprint" ""
			(at 189.23 73.66 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" ""
			(at 189.23 73.66 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Power symbol creates a global label with name \"GND\" , ground"
			(at 189.23 73.66 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "1"
			(uuid "3d9dcb19-1750-4402-9a06-d5fa39348f48")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "#PWR09")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "power:GND")
		(at 120.65 134.62 180)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "532e1c8a-aa90-427d-b2da-deb734f33a20")
		(property "Reference" "#PWR01"
			(at 120.65 128.27 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Value" "GND"
			(at 120.65 129.54 0)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Footprint" ""
			(at 120.65 134.62 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" ""
			(at 120.65 134.62 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Power symbol creates a global label with name \"GND\" , ground"
			(at 120.65 134.62 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "1"
			(uuid "b809b60e-5eb4-42ac-93c4-6355bc3dc5a2")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "#PWR01")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "Connector:Conn_01x01_Socket")
		(at 87.63 81.28 90)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "6bb54174-542e-486f-83d7-70495d2f62ec")
		(property "Reference" "J7"
			(at 88.9 80.6449 90)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify right)
			)
		)
		(property "Value" "Conector sensor lateral Esquerda"
			(at 88.9 83.1849 90)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify right)
			)
		)
		(property "Footprint" "Connector_PinSocket_2.54mm:PinSocket_1x01_P2.54mm_Vertical"
			(at 87.63 81.28 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" "~"
			(at 87.63 81.28 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Generic connector, single row, 01x01, script generated"
			(at 87.63 81.28 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "1"
			(uuid "7d72f6f5-b5af-4525-bea8-7cad2cba7f6d")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "J7")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "Regulator_Linear:LM7805_TO220")
		(at 177.8 55.88 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "7131bf56-c14b-4943-876d-a11060ee0c91")
		(property "Reference" "U2"
			(at 177.8 49.53 0)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Value" "LM7805_TO220"
			(at 177.8 52.07 0)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Footprint" "Package_TO_SOT_THT:TO-220-3_Vertical"
			(at 177.8 50.165 0)
			(effects
				(font
					(size 1.27 1.27)
					(italic yes)
				)
				(hide yes)
			)
		)
		(property "Datasheet" "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF"
			(at 177.8 57.15 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Positive 1A 35V Linear Regulator, Fixed Output 5V, TO-220"
			(at 177.8 55.88 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "2"
			(uuid "2db7ce53-38cf-46be-99f5-6efb40a3f459")
		)
		(pin "1"
			(uuid "79998bd7-ad44-45dd-bb1a-a1ac5257e31e")
		)
		(pin "3"
			(uuid "041c8939-221a-4a54-9c78-e751c25e742e")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "U2")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "power:GND")
		(at 95.25 68.58 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "782fba07-d286-44b2-bd07-6d57802f9a30")
		(property "Reference" "#PWR08"
			(at 95.25 74.93 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Value" "GND"
			(at 95.25 73.66 0)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Footprint" ""
			(at 95.25 68.58 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" ""
			(at 95.25 68.58 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Power symbol creates a global label with name \"GND\" , ground"
			(at 95.25 68.58 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "1"
			(uuid "b0736d1b-f79a-483e-a59f-910906ac8684")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "#PWR08")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "Connector:Conn_01x02_Socket")
		(at 191.77 99.06 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "858fc25f-91f2-48ba-a7a6-341c0c15f5a4")
		(property "Reference" "J3"
			(at 193.04 99.0599 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify left)
			)
		)
		(property "Value" "Motor 1"
			(at 193.04 101.5999 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify left)
			)
		)
		(property "Footprint" "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical"
			(at 191.77 99.06 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" "~"
			(at 191.77 99.06 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Generic connector, single row, 01x02, script generated"
			(at 191.77 99.06 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "2"
			(uuid "840a4a22-e9f7-43aa-9da3-ea7d11aec185")
		)
		(pin "1"
			(uuid "b24a3802-0c68-460b-8caf-e3faf267ab9f")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "J3")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "power:GND")
		(at 59.69 81.28 270)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "8ac6893c-fe05-43a1-a252-af32555e191f")
		(property "Reference" "#PWR03"
			(at 53.34 81.28 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Value" "GND"
			(at 55.88 81.2799 90)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify right)
			)
		)
		(property "Footprint" ""
			(at 59.69 81.28 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" ""
			(at 59.69 81.28 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Power symbol creates a global label with name \"GND\" , ground"
			(at 59.69 81.28 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "1"
			(uuid "b8db3dd4-f502-49d3-a7f6-112832f696ce")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "#PWR03")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "Connector:Conn_01x11_Pin")
		(at 29.21 101.6 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(uuid "8c44731e-d41e-4f3f-ab39-56eeceb66081")
		(property "Reference" "J1"
			(at 29.845 83.82 0)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Value" "Conn_01x11_Pin"
			(at 29.845 86.36 0)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Footprint" "Connector_PinSocket_2.54mm:PinSocket_1x11_P2.54mm_Vertical"
			(at 29.21 101.6 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" "~"
			(at 29.21 101.6 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Generic connector, single row, 01x11, script generated"
			(at 29.21 101.6 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "8"
			(uuid "86c34d0a-cd48-4df9-95ed-4803543e8a6a")
		)
		(pin "4"
			(uuid "d49fcfec-dc53-4f15-8c53-51e5f03d6b71")
		)
		(pin "2"
			(uuid "4b853e5b-3624-4ce1-a7bc-cf50b8f37171")
		)
		(pin "9"
			(uuid "02a5cb83-cbf6-4dc6-a5d9-fbb0ea25ae19")
		)
		(pin "1"
			(uuid "d009e001-335a-4b30-a8b1-e5eb2169d4b4")
		)
		(pin "5"
			(uuid "324cd1bd-5341-496d-af62-982a4b727063")
		)
		(pin "6"
			(uuid "f92aade2-e270-452f-8999-45418522ce74")
		)
		(pin "10"
			(uuid "3c7d2ca8-5aca-4c89-aa21-67398722de94")
		)
		(pin "7"
			(uuid "d192bdee-dea7-4833-a9ae-dc9656011051")
		)
		(pin "3"
			(uuid "44f78cdf-538a-46a4-b5d0-f600b63ca649")
		)
		(pin "11"
			(uuid "8ea284c0-11cb-449d-941c-14e891978c67")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "J1")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "power:GND")
		(at 177.8 72.39 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "8e96dffc-fbed-48e3-92c4-4b645fb8ba9d")
		(property "Reference" "#PWR05"
			(at 177.8 78.74 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Value" "GND"
			(at 180.34 73.6599 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify left)
			)
		)
		(property "Footprint" ""
			(at 177.8 72.39 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" ""
			(at 177.8 72.39 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Power symbol creates a global label with name \"GND\" , ground"
			(at 177.8 72.39 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "1"
			(uuid "a9c25370-5130-4218-acea-24fe6ac99bbf")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "#PWR05")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "Switch EG1213:EG1213")
		(at 189.23 86.36 270)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "8f4e05d4-8e33-4b6c-aeae-86fc7f65c545")
		(property "Reference" "S1"
			(at 194.31 85.0899 90)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify left)
			)
		)
		(property "Value" "EG1213"
			(at 194.31 87.6299 90)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify left)
			)
		)
		(property "Footprint" "EG1213:SW_EG1213"
			(at 180.086 85.344 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify bottom)
				(hide yes)
			)
		)
		(property "Datasheet" ""
			(at 189.23 86.36 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" ""
			(at 189.23 86.36 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "MF" "E-Switch"
			(at 184.15 86.106 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify bottom)
				(hide yes)
			)
		)
		(property "Description_1" "\nSlide Switch, EG Series, SPDT, Non-Shorting, ON-ON, 200mA DC, 30VDC, R/A, PC | E-Switch EG1213\n"
			(at 167.132 91.186 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify bottom)
				(hide yes)
			)
		)
		(property "Package" "None"
			(at 189.23 86.36 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify bottom)
				(hide yes)
			)
		)
		(property "Price" "None"
			(at 189.23 86.36 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify bottom)
				(hide yes)
			)
		)
		(property "Check_prices" "https://www.snapeda.com/parts/EG1213/E-Switch/view-part/?ref=eda"
			(at 202.184 86.614 0)
			(do_not_autoplace yes)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify bottom)
				(hide yes)
			)
		)
		(property "SnapEDA_Link" "https://www.snapeda.com/parts/EG1213/E-Switch/view-part/?ref=snap"
			(at 172.212 87.376 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify bottom)
				(hide yes)
			)
		)
		(property "MP" "EG1213"
			(at 189.23 86.36 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify bottom)
				(hide yes)
			)
		)
		(property "Purchase-URL" "https://www.snapeda.com/api/url_track_click_mouser/?unipart_id=13820&manufacturer=E-Switch&part_name=EG1213&search_term=None"
			(at 149.352 85.09 0)
			(do_not_autoplace yes)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify bottom)
				(hide yes)
			)
		)
		(property "Availability" "In Stock"
			(at 190.246 101.346 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify bottom)
				(hide yes)
			)
		)
		(property "MANUFACTURER" "E-Switch"
			(at 184.658 100.838 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify bottom)
				(hide yes)
			)
		)
		(pin "3"
			(uuid "50852e03-4899-4079-84ab-ba097c0cb6fc")
		)
		(pin "2"
			(uuid "d04282e9-7292-4c2b-94db-b5ea6dbceddd")
		)
		(pin "1"
			(uuid "5478ef21-98ab-45f8-9ba5-298b08f291d9")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "S1")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_name "Conn_01x10_Pin_1")
		(lib_id "Connector:Conn_01x10_Pin")
		(at 181.61 66.04 270)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(uuid "a14cfc8e-e27d-4b6e-90c0-5549dbeab5b2")
		(property "Reference" "Bateria1"
			(at 190.5 60.96 90)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Value" "Entrada bateria"
			(at 190.754 63.5 90)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Footprint" "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical"
			(at 181.61 66.04 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" "~"
			(at 181.61 66.04 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Generic connector, single row, 01x10, script generated"
			(at 181.61 66.04 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "1"
			(uuid "1c06536d-a023-44a6-b576-2d24fb77b6f6")
		)
		(pin "2"
			(uuid "ec6517ac-9fe4-4a97-94c8-eb1b5d7457b7")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "Bateria1")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_name "Conn_01x02_Socket_1")
		(lib_id "Connector:Conn_01x02_Socket")
		(at 44.45 143.51 180)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "a824e8e4-7a88-428b-8cab-a814e26c7821")
		(property "Reference" "J9"
			(at 45.085 135.89 0)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Value" "VCC pinos"
			(at 45.085 138.43 0)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Footprint" "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical"
			(at 44.45 143.51 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" "~"
			(at 44.45 143.51 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Generic connector, single row, 01x02, script generated"
			(at 44.45 143.51 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "2"
			(uuid "c83120d7-5e66-476b-9f19-2d92f32f7f4d")
		)
		(pin "1"
			(uuid "9bc82025-05e5-4ede-a59f-a4f0ba5463dd")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "J9")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "Connector:Conn_01x02_Socket")
		(at 194.31 114.3 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "abd07b89-8bcf-4f0b-8893-2c5017d494a6")
		(property "Reference" "J2"
			(at 195.58 114.2999 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify left)
			)
		)
		(property "Value" "Motor 1"
			(at 195.58 116.8399 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify left)
			)
		)
		(property "Footprint" "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical"
			(at 194.31 114.3 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" "~"
			(at 194.31 114.3 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Generic connector, single row, 01x02, script generated"
			(at 194.31 114.3 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "2"
			(uuid "2d93dd55-fae1-421d-a76e-7854cec4feaf")
		)
		(pin "1"
			(uuid "2f42c0d7-a76c-4bd4-9be8-5c5d6b8c0b11")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "J2")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "power:GND")
		(at 66.04 78.74 180)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "aeae7e86-61b4-4831-b926-635567b24740")
		(property "Reference" "#PWR06"
			(at 66.04 72.39 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Value" "GND"
			(at 66.04 73.66 0)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Footprint" ""
			(at 66.04 78.74 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" ""
			(at 66.04 78.74 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Power symbol creates a global label with name \"GND\" , ground"
			(at 66.04 78.74 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "1"
			(uuid "2becc059-2c99-4fb6-9c1e-285989476aa4")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "#PWR06")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "Connector:Conn_01x04_Socket")
		(at 74.93 54.61 90)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "b4f05edf-38d8-4356-bc89-1e9cc0d0696e")
		(property "Reference" "J5"
			(at 76.2 49.53 90)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Value" "Sensor Lateral - Esquerda"
			(at 76.2 52.07 90)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Footprint" "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Horizontal"
			(at 74.93 54.61 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" "~"
			(at 74.93 54.61 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Generic connector, single row, 01x04, script generated"
			(at 74.93 54.61 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "1"
			(uuid "17bdea25-a16f-484f-a8c3-e05e85d1e68c")
		)
		(pin "2"
			(uuid "196922a7-edaf-461f-9a15-d2743f490e8c")
		)
		(pin "3"
			(uuid "a4499db9-6dc5-4f38-a65c-866bb69dcaca")
		)
		(pin "4"
			(uuid "de9c40df-ee5c-45f6-9c34-34f2b4459545")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "J5")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "power:GND")
		(at 60.96 78.74 180)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "b5a93dc0-f811-4f63-ade8-339c73663750")
		(property "Reference" "#PWR02"
			(at 60.96 72.39 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Value" "GND"
			(at 60.96 73.66 0)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Footprint" ""
			(at 60.96 78.74 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" ""
			(at 60.96 78.74 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Power symbol creates a global label with name \"GND\" , ground"
			(at 60.96 78.74 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "1"
			(uuid "a9e7292e-d4d0-4734-a14b-b2b4de727047")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "#PWR02")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "power:GND")
		(at 34.29 114.3 90)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "b77e6836-4184-4a0a-b5ce-ff15d3cc0fcd")
		(property "Reference" "#PWR04"
			(at 40.64 114.3 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Value" "GND"
			(at 38.1 114.2999 90)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify right)
			)
		)
		(property "Footprint" ""
			(at 34.29 114.3 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" ""
			(at 34.29 114.3 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Power symbol creates a global label with name \"GND\" , ground"
			(at 34.29 114.3 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "1"
			(uuid "8a82072c-8ba6-476d-9ea5-4715700d6238")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "#PWR04")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "power:GND")
		(at 74.93 59.69 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "cad3482d-18e3-40a7-aea1-7e8981af9f47")
		(property "Reference" "#PWR07"
			(at 74.93 66.04 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Value" "GND"
			(at 74.93 64.77 0)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Footprint" ""
			(at 74.93 59.69 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" ""
			(at 74.93 59.69 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Power symbol creates a global label with name \"GND\" , ground"
			(at 74.93 59.69 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "1"
			(uuid "c8db503a-8d9c-4d4e-a89d-b2f014a35c76")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "#PWR07")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "ROB-Board-14450:ROB-14450")
		(at 148.59 106.68 0)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(uuid "d1a51e18-f624-4882-a5a0-393857ae7a42")
		(property "Reference" "U1"
			(at 148.59 86.36 0)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Value" "ROB-14450"
			(at 148.59 88.9 0)
			(effects
				(font
					(size 1.27 1.27)
				)
			)
		)
		(property "Footprint" "ROB_14450:MODULE_ROB-14450"
			(at 143.51 106.68 0)
			(show_name yes)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify bottom)
			)
		)
		(property "Datasheet" ""
			(at 148.59 106.68 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" ""
			(at 148.59 106.68 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "PARTREV" "11-13-17"
			(at 148.59 106.68 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify bottom)
				(hide yes)
			)
		)
		(property "MANUFACTURER" "Sparkfun Electronics"
			(at 148.59 106.68 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify bottom)
				(hide yes)
			)
		)
		(property "STANDARD" "Manufacturer Recommendation"
			(at 148.59 106.68 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify bottom)
				(hide yes)
			)
		)
		(pin "14"
			(uuid "d97cf15b-c8a0-4344-87c7-785a57674698")
		)
		(pin "16"
			(uuid "ee426336-cc2e-4111-821c-8ee5527c07dc")
		)
		(pin "15"
			(uuid "9a75a401-7370-4113-bae3-454ad683e89a")
		)
		(pin "2"
			(uuid "df5eaf39-c54a-44f3-8e63-e59482137994")
		)
		(pin "13"
			(uuid "e84c1692-6b7a-4bd8-aa88-746efa1d96e2")
		)
		(pin "8"
			(uuid "fbf89cfb-812b-40e0-8e7b-7bf56de031e0")
		)
		(pin "5"
			(uuid "4c230444-9687-4ad1-8c4b-4b9aa808b739")
		)
		(pin "12"
			(uuid "904c1725-32e6-48d8-b3f5-b77f03d30ec4")
		)
		(pin "1"
			(uuid "f8199771-de29-40d6-8ce2-34568ed9c555")
		)
		(pin "9"
			(uuid "743c00e8-da86-4fba-8ac2-00cf08d15253")
		)
		(pin "7"
			(uuid "f83edb74-8c67-4160-aa43-1a3d68803bae")
		)
		(pin "11"
			(uuid "b5139581-de29-43dd-a954-23dd33ad84eb")
		)
		(pin "3"
			(uuid "b22ab49f-398c-4d5e-9f47-1fa42c59d3ac")
		)
		(pin "4"
			(uuid "d44d8c55-edac-434d-91b4-1dbcdb20a68c")
		)
		(pin "10"
			(uuid "ce51a6bd-2508-4210-92a3-1cc4aa3de44e")
		)
		(pin "6"
			(uuid "9de20cbe-c2c0-4be2-8537-9c30cecef13e")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "U1")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "power:GND")
		(at 163.83 119.38 90)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "e0d11cc3-9f3b-4d9d-be91-671853a85a3b")
		(property "Reference" "#PWR010"
			(at 170.18 119.38 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Value" "GND"
			(at 167.64 119.3799 90)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify right)
			)
		)
		(property "Footprint" ""
			(at 163.83 119.38 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Datasheet" ""
			(at 163.83 119.38 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Power symbol creates a global label with name \"GND\" , ground"
			(at 163.83 119.38 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "1"
			(uuid "908187bb-a16e-4d54-93c8-7507a9c44b38")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "#PWR010")
					(unit 1)
				)
			)
		)
	)
	(symbol
		(lib_id "MCU_Module:Arduino_Nano_v2.x")
		(at 64.77 106.68 180)
		(unit 1)
		(exclude_from_sim no)
		(in_bom yes)
		(on_board yes)
		(dnp no)
		(fields_autoplaced yes)
		(uuid "f6cb6d21-0493-425d-ad20-ae0b07a5dd27")
		(property "Reference" "A2"
			(at 67.7865 78.74 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify right)
			)
		)
		(property "Value" "Arduino_Nano_v2.x"
			(at 67.7865 81.28 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(justify right)
			)
		)
		(property "Footprint" "Module:Arduino_Nano"
			(at 64.77 106.68 0)
			(effects
				(font
					(size 1.27 1.27)
					(italic yes)
				)
				(hide yes)
			)
		)
		(property "Datasheet" "https://www.arduino.cc/en/uploads/Main/ArduinoNanoManual23.pdf"
			(at 64.77 106.68 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(property "Description" "Arduino Nano v2.x"
			(at 64.77 106.68 0)
			(effects
				(font
					(size 1.27 1.27)
				)
				(hide yes)
			)
		)
		(pin "15"
			(uuid "5f4a578d-b2c2-4c57-8702-975ea3aaa1cc")
		)
		(pin "18"
			(uuid "472c8d2b-6f2a-430c-8135-1036f31eeef3")
		)
		(pin "4"
			(uuid "b607fbbe-afce-48bd-a51f-a98b413dae8f")
		)
		(pin "13"
			(uuid "2c1ab10d-1887-4f52-b415-d37f717f7c27")
		)
		(pin "30"
			(uuid "d794644a-152a-4865-bc7d-e652b97b37e9")
		)
		(pin "10"
			(uuid "3321eaea-83f2-4fea-9910-5cc6299daa3d")
		)
		(pin "19"
			(uuid "9d18da2c-a24f-412f-a389-7743fdeba348")
		)
		(pin "22"
			(uuid "03a80c0e-9e66-407e-8844-5b2f8ac11cb5")
		)
		(pin "12"
			(uuid "46d1f93e-5e0e-4edf-a8b8-478d00d48f87")
		)
		(pin "16"
			(uuid "50c767b5-16aa-49c7-8bb2-c6ff12480e60")
		)
		(pin "24"
			(uuid "b067560c-516c-4af5-8657-7cdb22549846")
		)
		(pin "7"
			(uuid "b155babd-f345-4f0a-bf74-620c88f03d9a")
		)
		(pin "27"
			(uuid "08eb99f1-9529-4dd5-acb0-78b2a1422436")
		)
		(pin "1"
			(uuid "4231951f-6a6b-42f4-801a-6557326bc17c")
		)
		(pin "11"
			(uuid "63acdcfd-b39c-4dd3-8e23-3df66f61da0c")
		)
		(pin "29"
			(uuid "135c7ef8-8da5-405b-8943-5b57ace3362d")
		)
		(pin "2"
			(uuid "3926f4d8-0a8e-4995-a706-436f6d9b5007")
		)
		(pin "5"
			(uuid "de896721-4737-4881-ba95-d43e76f6371e")
		)
		(pin "17"
			(uuid "f95ba8e6-8f34-444f-b6ad-d40354d48628")
		)
		(pin "28"
			(uuid "45899925-c15a-4c4e-89ca-4b86ec329ec6")
		)
		(pin "21"
			(uuid "323bf3d4-0991-4aa6-be90-4d306969f5f0")
		)
		(pin "6"
			(uuid "93ffea5b-ffb1-4a47-bd69-2861a7574c2e")
		)
		(pin "3"
			(uuid "71825d0b-bcdb-4037-acb9-5d5396287e16")
		)
		(pin "25"
			(uuid "9800a996-0c7b-4f49-8262-897f2c04eafd")
		)
		(pin "8"
			(uuid "472d5a52-ff78-4348-9574-73fa93b26b78")
		)
		(pin "9"
			(uuid "50c2e168-dfc4-47be-acff-5f2ed7810a1d")
		)
		(pin "20"
			(uuid "5dd6472d-a0d1-4e96-9950-c99da402d902")
		)
		(pin "26"
			(uuid "375dfd89-fc6e-43e9-9dec-0d542489cda8")
		)
		(pin "14"
			(uuid "473a530b-eac8-4a50-99e9-cc112680672d")
		)
		(pin "23"
			(uuid "10ce3367-06bf-44c2-a619-7553d31edb46")
		)
		(instances
			(project "LineFollowerRCX"
				(path "/84fbcbd8-9b55-4c71-8663-267634a18e15"
					(reference "A2")
					(unit 1)
				)
			)
		)
	)
	(sheet_instances
		(path "/"
			(page "1")
		)
	)
)