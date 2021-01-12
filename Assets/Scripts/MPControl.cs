
using System.Collections;
using System.Collections.Generic; using UnityEngine;

public class MPControl : MonoBehaviour{
    public float PositionReference_y=0;
    public Natural natural;
    public int GMRES_RepeatTime=2;
    public int PredictionTime=10;
    public float StableConstant=100;
    public GameObject InputObject;
    public GameObject Object;
    public GameObject PredictivePositionIndicaterSample;
    public float VelocityConstant, PositionConstant, NaturalPositionConstant,VelocityConstant_Stage, PositionConstant_Stage, NaturalPositionConstant_Stage;
    public float FinalEvaluarionScope=0.5f;//[s]
    GameObject[] PredictivePositionIndicater;
    Transform[] PredictivePositionIndicaterTransform;
    Transform InputTransform;
    Transform ObjectTransform;
    float[] ObjectPosition_y;
    float[] ObjectVelocity_y;
    float[] InputPosition_y;
    float[] DifferentialInputPosition_y;    
    float[,] AdjointVector;//Adjoint Vector[i,0] and [1,1], for ObjectPosition and ObjectVelocity
    float MinEvaluatingFunctionValue;
    float PreviousObjectPosition_y=0;
    float NaturalLength;
    float SpringConstent;
    float InitialTime;

    // Start is called before the first frame update
    void Start(){
        PredictivePositionIndicater=new GameObject[PredictionTime+1];
        PredictivePositionIndicaterTransform=new Transform[PredictionTime+1]; ObjectPosition_y=new float[PredictionTime+1];
        ObjectVelocity_y=new float[PredictionTime+1]; 
        InputPosition_y=new float[PredictionTime+1]; 
        DifferentialInputPosition_y=new float[PredictionTime+1]; 
        AdjointVector=new float[PredictionTime+1,2]; 
        InputTransform=InputObject.GetComponent<Transform>(); 
        ObjectTransform=Object.GetComponent<Transform>(); 
        NaturalLength=natural.NaturalLength; 
        SpringConstent=natural.SpringConst;

        for(int i = 0; i <PredictionTime+1 ; i++){
            ObjectPosition_y[i]=0; 
            ObjectVelocity_y[i]=0;
            InputPosition_y[i]=0; 
            DifferentialInputPosition_y[i]=0; 
            AdjointVector[i,0]=0; 
            AdjointVector[i,1]=0; 
            PredictivePositionIndicater[i]=Instantiate (PredictivePositionIndicaterSample); 
            PredictivePositionIndicaterTransform[i]=PredictivePositionIndicater[i].GetComponent<Transform>();
        }  
        ObjectPosition_y[0]=ObjectTransform.position.y; 
        ObjectVelocity_y[0]=0;        
        InputPosition_y[0]=ObjectPosition_y[0]+NaturalLength; 
        for(int i=1;i<PredictionTime;i++)InputPosition_y[i]=InputPosition_y[0]; 
        PreviousObjectPosition_y=ObjectPosition_y[0]; 
        InitialTime=Time.time;
    }
    // Update is called once per frame
    void FixedUpdate(){
        InputTransform.position=new Vector3(0,InputPosition_y[0],0);
        float dt=Time.deltaTime; 
        ObjectPosition_y[0]=ObjectTransform.position.y; 
        ObjectVelocity_y[0]=(ObjectPosition_y[0]-PreviousObjectPosition_y)/dt; 
        PreviousObjectPosition_y=ObjectPosition_y[0]; 
        float EvaluationDeltaTime=FinalEvaluarionScope*(1-Mathf.Exp(-2.5f*(Time.time-InitialTime+0.01f)))/PredictionTime; 
        for(int i=1;i<PredictionTime+1; i++) {
            ObjectPosition_y[i]=ObjectPosition_y[i-1]+ObjectVelocity_y[i-1]*EvaluationDeltaTime; 
            ObjectVelocity_y[i]=ObjectVelocity_y[i-1]-SpringConstent*(InputPosition_y[i-1] -ObjectPosition_y[i-1]-NaturalLength)*EvaluationDeltaTime;
        }

        AdjointVector[PredictionTime,0]=PositionConstant*(ObjectPosition_y[PredictionTime] -PositionReference_y)
                                -NaturalPositionConstant*(InputPosition_y[PredictionTime] -ObjectPosition_y[PredictionTime] - NaturalLength); 
        AdjointVector[PredictionTime, 1]=VelocityConstant*(ObjectVelocity_y[PredictionTime]); 

        for(int i=PredictionTime-1;i>0;i--){ 
            AdjointVector[i,0]=AdjointVector[i+1,0]+PositionConstant_Stage*(ObjectPosition_y[i]-PositionReference_y)
                                    -NaturalPositionConstant_Stage*(InputPosition_y[i]-ObjectPosition_y[i]-NaturalLength)
                                                                                            +AdjointVector[i+1,1]*SpringConstent; 
            AdjointVector[i,1]=AdjointVector[i+1,1]+VelocityConstant_Stage*ObjectVelocity_y[i]+AdjointVector[i+1,0];
        }
        float[,] Difference=new float[PredictionTime, PredictionTime];//because ri is vector 
        float[] DifferenceInnerProduct=new float[PredictionTime-1]; 
        float[,] OrthogonalBasis=new float[PredictionTime, PredictionTime]; 

        for(int i=0;i<PredictionTime;i++) {
            Difference[0,i]=-StableConstant*(NaturalPositionConstant_Stage*(InputPosition_y[i]-ObjectPosition_y[i]-NaturalLength)
                                                                -AdjointVector[i+1,1]*SpringConstent) -((NaturalPositionConstant_Stage*(InputPosition_y[i]+DifferentialInputPosition_y[i]*EvaluationDeltaTime-ObjectPosition_y[i]-ObjectVelocity_y[i]*EvaluationDeltaTime-NaturalLength)
                                                                -AdjointVector[i+1,1]*SpringConstent)-(NaturalPositionConstant_Stage*(InputPosition_y[i]-ObjectPosition_y[i]-NaturalLength)-AdjointVector[i+1,1]*SpringConstent))/EvaluationDeltaTime; 
            DifferenceInnerProduct[0]+=Mathf.Pow(Difference[0,i], 2);
            Debug.Log(AdjointVector[i+1,1]*SpringConstent);
        }

        for(int i=0;i<PredictionTime; i++) OrthogonalBasis[0,i]=Difference[0,i]/DifferenceInnerProduct[0];

        float[,] h=new float[GMRES_RepeatTime+1, GMRES_RepeatTime];//gyo, retu 
        float[] y=new float[GMRES_RepeatTime]; 
        for(int i=0;i<GMRES_RepeatTime+1; i++){ 
            for(int j=0;j<GMRES_RepeatTime; j++){
                h[i,j]=0;
                y[j]=0;
            }
        }
        for(int i=0;i<GMRES_RepeatTime; i++){ 
            for(int j=0; j<PredictionTime; j++) OrthogonalBasis[i+1,j]=((NaturalPositionConstant_Stage*(InputPosition_y[j]+OrthogonalBasis[i,j]*EvaluationDeltaTime
                                            -ObjectPosition_y[j]-ObjectVelocity_y[j]*EvaluationDeltaTime - NaturalLength)-AdjointVector[j+1,1]*SpringConstent 
                                            -(NaturalPositionConstant_Stage*(InputPosition_y[j]-ObjectPosition_y[j]-NaturalLength)-AdjointVector[j+1,1]*SpringConstent)))/EvaluationDeltaTime; 
            for(int j=0; j<i+1;j++){
                for(int k=0;k<PredictionTime;k++) h[j,i]+=OrthogonalBasis[i+1,k] *OrthogonalBasis[j,k]; 
                for(int k=0; k<PredictionTime; k++) OrthogonalBasis[i+1, k]=OrthogonalBasis[i+1,k]-h[j,i]*OrthogonalBasis[j,k];
            }
            for(int j=0; j<PredictionTime; j++)h[i+1,i]+=Mathf.Pow(OrthogonalBasis[i+1,j],2); 
            for(int j=0; j<PredictionTime; j++) OrthogonalBasis[i+1,j]=OrthogonalBasis[i+1, j]/h[i+1,i];
        }
        float[,] TotalGivens=new float[GMRES_RepeatTime+1, GMRES_RepeatTime+1];
        for(int i=0;i<GMRES_RepeatTime+1; i++) { 
            for(int j=0; j<GMRES_RepeatTime+1; j++){
                TotalGivens[i,j]=1;
            }
        }

        for(int i=0;i<GMRES_RepeatTime; i++){
            float r=Mathf.Sqrt(Mathf.Pow(h[i+1,i]*Mathf.Pow(10,-10),2)+Mathf.Pow(h[i,i]*Mathf.Pow(10,-10),2));//sqrt((h*10^(-10))^2+(^2) 
            float SinTheta=h[i+1,i]/(r*Mathf.Pow(10,10)); 
            float CosTheta=-h[i,i]/(r*Mathf.Pow(10,10));

            for(int j=0; j<GMRES_RepeatTime+1; j++){ 
                for(int k=0; k<GMRES_RepeatTime+1;k++){ 
                    if(k<GMRES_RepeatTime) {
                        if(j==i)h[j,k]=h[j,k] *CosTheta-h[j+1,k]*SinTheta; 
                        else if(j==i+1 && k==i)h[j,k]=0; 
                        else if(j==i+1 && k!=i)h[j, k]=h[j-1,k]*SinTheta+h[j,k]*CosTheta;
                    }
                    float TimesValue=1; 
                    if(k==i){
                        if(j==i+1) TimesValue=SinTheta; 
                        else if(j<i+1)TimesValue=CosTheta;
                        else if(j>i+1)TimesValue=0; 
                    }else if(k==i+1){
                        if(j==i+1)TimesValue=CosTheta; 
                        else if(j<i+1)TimesValue=-SinTheta; 
                        else TimesValue=1;
                    }
                    TotalGivens[j,k]*=TimesValue; //this would be a inefficient way
                }
            }
        }
    }
}
