
using System.Collections;
using System.Collections.Generic; 
using UnityEngine;
//using DifiningObjects_MPControl;

/*
    こんにちは。たこすです。MPCについて学んでみたので、C/GMRES法を用いたMPCによるばね振子制御をUnityでシミュレーションしてみました。
    このファイルのコードが計算をしています。

    参考文献
    非線形最適制御入門 /大塚敏行 著
    
    前置き
    コメントアウト機能を利用して各式の(申し訳程度の)説明を行っているが、その時に利用する文字等について先に示しておく
    t:そのループ時の時刻
    τ:予測範囲での時刻、時刻tのときτ=0である
    d~:微小な~ ex:dx/dtはxの時間微分表す
    ð~:ex:ðx/ðuはxのuによる偏微分を表す
    x[t]:時刻tにおける目標物の位置
    v[t]:時刻tにおける目標物の速度
    u[t]:時刻tにおける入力

    パラメータ記録
    -2021/jan/16　収束しない&挙動が不安定だが頑張っている様子(評価対象：Object位置、速度、自然長からの距離)
    GMRES_RepeatConstant 2
    PredictionTime 10
    StableConstant 100
    FinalEvaluationScope 0.5
    NaturalPositionConstant -1.8 //-2.9~-1.8(-2.0を除く)で同じような挙動を確認
    それ以外 1
    -2021/jan/16　なんかいい感じ(評価対象：Object位置、速度、自然長からの距離)
    GMRES_RepeatConstant 2
    PredictionTime 10
    StableConstant 100
    FinalEvaluationScope 0.7
    NaturalPositionConstant -0.6//ここらへんはテキトーでいける
    NaturalPositionConstant_Stage -0.9
    VelocityConstant 0.1
    VelocityConstant_Stage 0.1
    それ以外 1
*/

public class MPControl : MonoBehaviour{
    public bool MPC_mode=true;
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
    public float[] ObjectPosition_y;
    float[] ObjectVelocity_y;
    public float[] InputPosition_y;
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
        //input InputPosition[0] in real input
        // u[τ=0]を実際にモデルに入力する
        if(MPC_mode) InputTransform.position=new Vector3(0,InputPosition_y[0],0);

        //measure real delta time (not evaluation delta time)
        //制御ループ周期(dt)測定
        float dt=Time.deltaTime; 

        //meature present Object's position and velocity and input them into ObjectPosition[0],ObjectVelocity[0]
        //目標物の位置と速度を計測し、x[τ=0],v[τ=0]に代入する
        ObjectPosition_y[0]=ObjectTransform.position.y; 
        ObjectVelocity_y[0]=(ObjectPosition_y[0]-PreviousObjectPosition_y)/dt; 
        PreviousObjectPosition_y=ObjectPosition_y[0]; 

        //difine EvaluationTime in this loop
        //EvalutionTime will converge to FinalEvaluationScope/PredictionTime
        //予測範囲を予測範囲分割数で割った予測空間内でのループ周期(dτ)を設定する
        //予め設定した最終予測範囲に収束するように設定する：ループ開始時は予測の精度が高くないため予測範囲は初めは0で0→最終予測範囲となるように
        float EvaluationDeltaTime=FinalEvaluarionScope*(1-Mathf.Exp(-2.5f*(Time.time-InitialTime+0.01f)))/PredictionTime; 

        //forsee ObjectPosition[i] and ObjectVelocity[i] using ObjectPosition[0] and ObjectVelocity[0], given InputPosition[i]
        //上で与えられたv[τ=0],x[τ=0](つまりv[t],x[t])とuからx[i],v[i]を順に予想していく
        for(int i=1;i<PredictionTime+1; i++) {
            float power=-SpringConstent*(InputPosition_y[i-1] -ObjectPosition_y[i-1]-NaturalLength);
            ObjectPosition_y[i]=ObjectPosition_y[i-1]+ObjectVelocity_y[i-1]*EvaluationDeltaTime+power*EvaluationDeltaTime*EvaluationDeltaTime/2; 
            ObjectVelocity_y[i]=ObjectVelocity_y[i-1]+power*EvaluationDeltaTime;
        }

        //calculate AdjointVector[i,0]and[i,1] :[i,0] for position, [i,1] for velocity
        //随伴変数を求める
        //at first, AdjointVector[last] is calculated by AdjointVector[last]=ð(TerminalCost)/ðx
        //初めに、予測範囲内で最終の随伴変数を求める、これは終端コストのx[N*dτ]での偏微分に等しい
        AdjointVector[PredictionTime,0]=PositionConstant*(ObjectPosition_y[PredictionTime] -PositionReference_y); 
        AdjointVector[PredictionTime, 1]=VelocityConstant*(ObjectVelocity_y[PredictionTime]); 

        //following AdjointVector[last], AdjointVector[last -1] can be calculated by AdjointVector[last -1]=AdjointVector[last]+ ðH/ðx, and so on.
        //
        for(int i=PredictionTime-1;i>0;i--){ 
            AdjointVector[i,0]=AdjointVector[i+1,0]+(PositionConstant_Stage*(ObjectPosition_y[i]-PositionReference_y)
                                    -NaturalPositionConstant_Stage*(InputPosition_y[i]-ObjectPosition_y[i]-NaturalLength)
                                                                                            +AdjointVector[i+1,1]*SpringConstent)*EvaluationDeltaTime; 
            AdjointVector[i,1]=AdjointVector[i+1,1]+(VelocityConstant_Stage*ObjectVelocity_y[i]+AdjointVector[i+1,0])*EvaluationDeltaTime;
        }

        //calculate dU/dt using GMRES method
        float[] Difference=new float[PredictionTime];//because ri is vector 
        float DifferenceInnerProduct=0; 
        float[,] OrthogonalBasis=new float[GMRES_RepeatTime+1, PredictionTime]; 

        for(int i=0;i<PredictionTime;i++) {
            Difference[i]=-StableConstant*(NaturalPositionConstant_Stage*(InputPosition_y[i]-ObjectPosition_y[i]-NaturalLength)-AdjointVector[i+1,1]*SpringConstent) 
                            -((NaturalPositionConstant_Stage*(InputPosition_y[i]+DifferentialInputPosition_y[i]*EvaluationDeltaTime-ObjectPosition_y[i]-ObjectVelocity_y[i]*EvaluationDeltaTime-NaturalLength)-AdjointVector[i+1,1]*SpringConstent)
                                -(NaturalPositionConstant_Stage*(InputPosition_y[i]-ObjectPosition_y[i]-NaturalLength)-AdjointVector[i+1,1]*SpringConstent))/EvaluationDeltaTime; 
            DifferenceInnerProduct+=Mathf.Pow(Difference[i], 2);//sqrt this later
        }
        DifferenceInnerProduct=Mathf.Sqrt(DifferenceInnerProduct);


        for(int i=0;i<PredictionTime; i++) OrthogonalBasis[0,i]=Difference[i]/DifferenceInnerProduct;

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
                                                    -ObjectPosition_y[j]-ObjectVelocity_y[j]*EvaluationDeltaTime - NaturalLength)-AdjointVector[j+1,1]*SpringConstent)
                                                    -(NaturalPositionConstant_Stage*(InputPosition_y[j]-ObjectPosition_y[j]-ObjectVelocity_y[j]*EvaluationDeltaTime-NaturalLength)-AdjointVector[j+1,1]*SpringConstent))/EvaluationDeltaTime; 
            for(int j=0; j<i+1;j++){
                for(int k=0;k<PredictionTime;k++) h[j,i]+=OrthogonalBasis[i+1,k]*OrthogonalBasis[j,k]; 
                for(int k=0;k<PredictionTime;k++) OrthogonalBasis[i+1,k]=OrthogonalBasis[i+1,k]-h[j,i]*OrthogonalBasis[j,k];
            }
            for(int j=0; j<PredictionTime; j++)h[i+1,i]+=Mathf.Pow(OrthogonalBasis[i+1,j],2); //sqrt this later
            h[i+1,i]=Mathf.Sqrt(h[i+1,i]);
            for(int j=0; j<PredictionTime; j++) OrthogonalBasis[i+1,j]=OrthogonalBasis[i+1,j]/h[i+1,i];
            
        }


        /*
        ここまでの計算により
        {       {
              +
            }       }
        */
        float[] GivensColumn_1=new float[GMRES_RepeatTime+1];
        for(int i=0;i<GMRES_RepeatTime+1; i++) GivensColumn_1[i]=1;

        for(int i=0;i<GMRES_RepeatTime; i++){
            float r=Mathf.Sqrt(Mathf.Pow(h[i+1,i],2)+Mathf.Pow(h[i,i],2));//sqrt((h*10^(-10))^2+(^2) 
            float SinTheta=h[i+1,i]/r; 
            float CosTheta=-h[i,i]/r;

            for(int j=0; j<GMRES_RepeatTime+1; j++){ 
                for(int k=0; k<GMRES_RepeatTime;k++){ 
                    if(j==i)h[j,k]=h[j,k]*CosTheta-h[j+1,k]*SinTheta; 
                    else if(j==i+1 && k==i)h[j,k]=0; 
                    else if(j==i+1 && k>i)h[j,k]=h[j-1,k]*SinTheta+h[j,k]*CosTheta;
                    /*float TimesValue=1; 
                    if(k==i){
                        if(j==i+1) TimesValue=SinTheta; 
                        else if(j<i+1)TimesValue=CosTheta;
                        else if(j>i+1)TimesValue=0; 
                    }else if(k==i+1){
                        if(j==i+1)TimesValue=CosTheta; 
                        else if(j<i+1)TimesValue=-SinTheta; 
                        else TimesValue=1;
                    }
                    GivensColumn_1[j,k]*=TimesValue; //this would be a inefficient way*/
                }
                if(j==i)GivensColumn_1[j]*=CosTheta;
                else if(j>i)GivensColumn_1[j]*=SinTheta;
            }
        }

        for(int i=GMRES_RepeatTime-1;i>0-1;i--) {
            float DevidedValue=GivensColumn_1[i]*DifferenceInnerProduct;
            for(int j=GMRES_RepeatTime-1;j>i;j--) DevidedValue-=h[i,j]*y[j];
            y[i]=DevidedValue/h[i,i];
        }

        //calculate U by U=(previous)U+dU/dt*dt
        for(int i=0;i<PredictionTime;i++){
            for(int j=0;j<GMRES_RepeatTime;j++)DifferentialInputPosition_y[i]+=OrthogonalBasis[j,i]*y[j];
            InputPosition_y[i]+=DifferentialInputPosition_y[i]*dt;///////////////////////////////
        }

        //move predictiove objet's position indicater
        for(int i=1;i<PredictionTime;i++)PredictivePositionIndicaterTransform[i].position=new Vector3(0,ObjectPosition_y[i],-0.1f);
        
        Debug.Log(ObjectPosition_y[0]);
    }
}
