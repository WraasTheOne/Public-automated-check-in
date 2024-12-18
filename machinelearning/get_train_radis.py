from readis_intercation import ReadStreamReadsOperation
import csv

def append_train_data(is_checkin="no", output_csv="train_data.csv"):
    readis = ReadStreamReadsOperation()
    token = "7a91f4aaf7d909d56992c8c17b5bf18d58fb7db12f9ec97dcf958ba0f229f18c"
    
    # Fetch data from the stream
    get_data = readis.read_stream(token)  
    print(get_data)

    # Open the CSV file in append mode
    with open(output_csv, mode='a', newline='') as csvfile:
        # Define the column headers (ensure consistency with the existing file)
        fieldnames = ['esp1id', 'rssi1', 'esp2id', 'rssi2', 'is_checkin', 'Passangercount']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        # Iterate over the stream data
        for entry_id, entry_data in get_data:
            # Decode byte data into strings
            row = {
                'esp1id': entry_data.get(b'esp1id', b'').decode('utf-8'),
                'rssi1': entry_data.get(b'rssi1', b'').decode('utf-8'),
                'esp2id': entry_data.get(b'esp2id', b'').decode('utf-8'),
                'rssi2': entry_data.get(b'rssi2', b'').decode('utf-8'),
                'is_checkin': is_checkin,  # Use the dynamic label
                "Passangercount": 1,

            }
            
            # Write the row to the CSV
            writer.writerow(row)
    
    print(f"Appended data with is_checkin='{is_checkin}' to {output_csv}")

if __name__ == "__main__":
    append_train_data(is_checkin="no", output_csv="train_data.csv")
    
