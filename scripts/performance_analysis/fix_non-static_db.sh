#!/bin/bash 

function print_help {
	echo
	echo "usage: fix_non-static_db.sh [-d <database file>]"
	echo
	echo "Update TENA sqlite databases that will not export in dataview"
	echo
	echo "optional arguments:"
	echo "    -d <database file> , --database <database file>   	input database file to be decoded"
	echo "    -h, --help             						            show help"
	echo
}

database_file="NONE"

for arg in "$@"
do
	if [ "$next_flag_is_database_file" = true ]; then

		database_file=$arg
		next_flag_is_database_file=false

	elif [[ $arg == "-d" ]] || [[ $arg == "--database" ]]; then
		
		next_flag_is_database_file=true

	elif [[ $arg == "--help" ]] || [[ $arg == "-h" ]]; then
		
		print_help
		exit

	elif [[ $arg != "" ]]; then
		
		echo
		echo "Invalid argument: $arg"
		print_help
		exit

	fi
done

if [[ $database_file == "NONE" ]]; then
	echo
	echo "Please specificy a database file"
	print_help
	exit
fi

# Ensure mysql-client is installed
REQUIRED_PKG="sqlite3"
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo Checking for $REQUIRED_PKG: $PKG_OK
if [ "" = "$PKG_OK" ]; then
  echo "No $REQUIRED_PKG found. Installing $REQUIRED_PKG."
  sudo apt-get update
  sudo apt-get --yes install $REQUIRED_PKG
fi

new_stop_time_value=1

# Run the SQLite query to update the StopTime value
sqlite3 "$database_file" <<EOF
UPDATE CollectionData
SET StopTime = $new_stop_time_value
WHERE rowid = 1;
EOF

echo
echo "Database updated successfully."