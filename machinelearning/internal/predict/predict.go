package predict

import (
	"fmt"

	"github.com/sjwhitworth/golearn/base"
	"github.com/sjwhitworth/golearn/ensemble"
	"github.com/sjwhitworth/golearn/evaluation"
)

var model *ensemble.RandomForest

// TrainModel trains a Random Forest model from a CSV file and saves it to disk
func TrainModel(csvFilePath string, modelSavePath string) error {
	// Load the data from CSV
	rawData, err := base.ParseCSVToInstances(csvFilePath, true)
	if err != nil {
		return fmt.Errorf("failed to parse CSV: %v", err)
	}

	// Split the dataset into training and testing sets
	trainData, testData := base.InstancesTrainTestSplit(rawData, 0.7)

	// Train a Random Forest model
	rf := ensemble.NewRandomForest(100, 3) // 100 trees, 3 random features per split
	err = rf.Fit(trainData)
	if err != nil {
		return fmt.Errorf("failed to train model: %v", err)
	}

	// Evaluate the model
	predictions, err := rf.Predict(testData)
	if err != nil {
		return fmt.Errorf("failed to make predictions: %v", err)
	}
	confusionMatrix, err := evaluation.GetConfusionMatrix(testData, predictions)
	if err != nil {
		return fmt.Errorf("failed to calculate confusion matrix: %v", err)
	}
	accuracy := evaluation.GetAccuracy(confusionMatrix)
	fmt.Printf("Model accuracy: %.2f%%\n", accuracy*100)

	// Save the model to disk
	err = base.SerializeToFile(modelSavePath, rf)
	if err != nil {
		return fmt.Errorf("failed to save model: %v", err)
	}

	return nil
}

// LoadModel loads a previously saved model into a global variable
func LoadModel(modelPath string) error {
	loadedModel, err := base.Load(modelPath)
	if err != nil {
		return fmt.Errorf("failed to load model: %v", err)
	}

	// Type assert to RandomForest
	rf, ok := loadedModel.(*ensemble.RandomForest)
	if !ok {
		return fmt.Errorf("loaded model is not a RandomForest")
	}

	model = rf
	return nil
}

// PredictVehicleStatus takes RSSI1, RSSI2, passenger count, and returns if in the vehicle
func PredictVehicleStatus(rssi1, rssi2 float64, passengerCount int) (string, error) {
	// Ensure the model is loaded
	if model == nil {
		return "", fmt.Errorf("model is not loaded")
	}

	// Create a test instance
	attributes := base.NewDenseInstances()
	attributes.Extend(base.NewRealAttribute("RSSI1"))
	attributes.Extend(base.NewRealAttribute("RSSI2"))
	attributes.Extend(base.NewRealAttribute("PassengerCount"))
	attributes.Extend(base.NewCategoricalAttribute("InOrNot", []string{"yes", "no"}))
	attributes.AddClassAttribute(base.NewCategoricalAttribute("InOrNot", []string{"yes", "no"}))

	row := []float64{rssi1, rssi2, float64(passengerCount)}
	instance := base.NewDenseInstance(attributes, row)

	// Predict using the loaded model
	prediction, err := model.Predict(instance)
	if err != nil {
		return "", fmt.Errorf("failed to make prediction: %v", err)
	}

	if prediction.String() == "yes" {
		return "yes", nil
	}
	return "no", nil
}
